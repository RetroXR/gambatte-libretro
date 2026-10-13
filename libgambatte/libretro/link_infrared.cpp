#include "link_infrared.h"

#include <stdlib.h>
#include <string.h>

#include "gambatte_log.h"

/* Names the emulated wire, and keeps it apart from the link cable's: the two
 * share a bus implementation and nothing else, and a core that attached its
 * infrared port to a serial lead would be refused here rather than heard. mGBA's
 * Game Boy speaks the same id and the same message, so a unit run by either core
 * sees a unit run by the other. */
#define IR_PROTOCOL "gbc-ir-1"

/* Twice 4194304, the Game Boy's normal CPU clock. See LinkSerial::now_. */
#define IR_CLOCK_RATE ((uint64_t)8388608)

/* How late a flash arrives, and how often two units in use rendezvous: 122
 * microseconds, which is 512 cycles of an undoubled CPU.
 *
 * Light takes no time and this is not light, so it is as small as it can be
 * afforded. Every edge is late by the same amount, which no receiver can see;
 * what a game CAN see is its partner's answers arriving twice this late, once
 * for each direction. The protocols leave room for that because the hardware
 * needed it -- Pokemon waits two milliseconds before it answers and eight
 * before it gives up -- and a game polling the port from a timer interrupt
 * cannot resolve much under thirty microseconds to begin with.
 *
 * Smaller is a rendezvous between two emulation threads that much more often. */
#define IR_LATENCY ((uint64_t)1024)

/* The horizon of a port nobody is using, about two milliseconds. A promise this
 * long is still standing when the port wakes, and is what the first flash after
 * that has to wait out -- once, at the head of a conversation, where every
 * protocol is waiting for a button to be pressed anyway. */
#define IR_IDLE_HORIZON ((uint64_t)16384)

/* How long after its last flash a port still counts as in use: an eighth of a
 * second. A unit that only ever transmits never switches its receiver on, so
 * the LED itself has to keep the horizon short between one flash and the next.
 * Also the silence across which an edge may go back to the ordinary latency. */
#define IR_HOLD ((uint64_t)1048576)

/* How long a lit LED keeps being announced after the view changes. A message to
 * a peer that has not rendezvoused since the bus was rebuilt has nowhere to land
 * and is dropped, and an LED left on says nothing further by itself. Counted on
 * this machine's own clock, never by whether an answer came: see LinkSerial. */
#define IR_REANNOUNCE (8 * IR_IDLE_HORIZON)

/* Bus message, packed by hand: see the note on LS_MSG_SIZE in link_serial.cpp. */
enum
{
	IR_LEVEL = 1
};

#define IR_MSG_SIZE 8

LinkInfrared::LinkInfrared()
	: link_(0)
	, port_(0)
	, handle_(0)
	, attached_(false)
	, deaf_(false)
	, selfId_(0)
	, peers_(0)
	, now_(0)
	, floor_(0)
	, lastRaw_(0)
	, haveRaw_(false)
	, horizon_(IR_IDLE_HORIZON)
	, promised_(0)
	, grant_(0)
	, anchored_(false)
	, reannounceUntil_(0)
	, rp_(0)
	, led_(false)
	, delay_(IR_LATENCY)
	, lastEdge_(0)
	, lastStamp_(0)
	, edgeSeen_(false)
	, lit_(0)
	, edgeHead_(0)
	, edgeCount_(0)
	, overflowed_(false)
	, edgesIn_(0)
	, edgesLate_(0)
	, worstLate_(0)
{
	memset(edges_, 0, sizeof(edges_));
}

LinkInfrared::~LinkInfrared()
{
	stop();
}

void LinkInfrared::setInterface(const struct retro_link_interface *link, unsigned port)
{
	link_ = link;
	port_ = port;
}

bool LinkInfrared::start()
{
	if (attached_)
		return true;
	if (!link_)
		return false;

	handle_ = link_->attach(port_, IR_PROTOCOL, IR_CLOCK_RATE);
	if (!handle_)
	{
		gambatte_log(RETRO_LOG_WARN, "Infrared: the frontend refused port %u\n", port_);
		return false;
	}

	attached_ = true;
	anchored_ = false;
	reannounceUntil_ = 0;
	forgetPeers();
	refreshPeers();
	gambatte_log(RETRO_LOG_INFO, "Infrared: attached to port %u, %u unit(s) in view\n",
	             port_, peers_);
	return true;
}

void LinkInfrared::stop()
{
	if (!attached_)
		return;

	if (getenv("GBC_IR_STATS"))
		gambatte_log(RETRO_LOG_WARN,
			"Infrared: %u edges in, %u of them late, the worst by %u ticks\n",
			edgesIn_, edgesLate_, (unsigned)worstLate_);

	link_->detach(handle_);
	attached_ = false;
	anchored_ = false;
	reannounceUntil_ = 0;
	handle_ = 0;
	peers_ = 0;
	selfId_ = 0;
	forgetPeers();
}

void LinkInfrared::forgetPeers()
{
	lit_ = 0;
	edgeHead_ = 0;
	edgeCount_ = 0;
}

bool LinkInfrared::hot() const
{
	/* Either half of the port: the receiver switched on, or the LED lit. And
	 * for a while after a flash, because a unit that only transmits writes
	 * nothing but that one bit and is dark between flashes. */
	if (rp_ & 0xC1)
		return true;
	return edgeSeen_ && now_ - lastEdge_ < IR_HOLD;
}

void LinkInfrared::begin(unsigned long cc, bool doubleSpeed)
{
	(void)doubleSpeed;

	lastRaw_ = cc;
	haveRaw_ = true;
	if (now_ < floor_)
		now_ = floor_;
}

void LinkInfrared::note(unsigned long cc, bool doubleSpeed)
{
	/* One tick a cycle in double speed and two outside it. A step backwards is
	 * the counter having been rebased, which begin() will already have covered
	 * for anything that runs: it is given up rather than guessed at. */
	if (haveRaw_ && cc > lastRaw_)
		now_ += (uint64_t)(cc - lastRaw_) << (doubleSpeed ? 0 : 1);

	lastRaw_ = cc;
	haveRaw_ = true;
}

void LinkInfrared::produced(unsigned samples)
{
	floor_ += (uint64_t)samples * 4;
	if (now_ < floor_)
		now_ = floor_;

	/* The counter may be rebased before the next chunk, and begin() reads it
	 * afresh either way. */
	haveRaw_ = false;
}

void LinkInfrared::wire(bool on)
{
	unsigned char msg[IR_MSG_SIZE];
	uint64_t stamp;

	/* Nothing may be sent before this machine has rendezvoused since the bus
	 * was last rebuilt: see LinkSerial::wire. Asking for nothing is granted at
	 * once, whatever anybody else is doing. */
	if (!anchored_)
	{
		uint64_t safe = now_ + horizon_;
		if (safe < promised_)
			safe = promised_;
		grant_ = link_->advance(handle_, now_, safe, 0, 0);
		promised_ = safe;
		anchored_ = true;
	}

	/* The promise made to the bus has to be kept, and it may reach further
	 * than the latency if it was made while the port was idle. Then the delay
	 * grows to meet it and stays grown: this edge and the next must be moved
	 * by the same amount. */
	stamp = now_ + delay_;
	if (stamp < promised_)
	{
		delay_ = promised_ - now_;
		stamp = promised_;
	}
	if (stamp < lastStamp_)
		stamp = lastStamp_;
	lastStamp_ = stamp;

	memset(msg, 0, sizeof(msg));
	msg[0] = IR_LEVEL;
	msg[1] = (unsigned char)selfId_;
	msg[2] = on ? 1 : 0;
	link_->send(handle_, stamp, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

void LinkInfrared::setLed(bool on)
{
	if (on == led_)
		return;
	led_ = on;

	/* A new burst, if the last edge was long enough ago that nothing can be
	 * timing the gap between the two. */
	if (!edgeSeen_ || now_ - lastEdge_ >= IR_HOLD)
		delay_ = IR_LATENCY;
	edgeSeen_ = true;
	lastEdge_ = now_;

	if (attached_ && peers_ >= 2)
		wire(on);
}

void LinkInfrared::write(unsigned long cc, bool doubleSpeed, unsigned char rp)
{
	note(cc, doubleSpeed);
	rp_ = rp;
	setLed((rp & 1) != 0);
}

void LinkInfrared::restored(unsigned char rp)
{
	/* No clock: the counter has just been replaced. The machine stands where
	 * the last chunk left it, which is where now_ already is. */
	haveRaw_ = false;
	rp_ = rp;
	setLed((rp & 1) != 0);
}

bool LinkInfrared::lit(unsigned long cc, bool doubleSpeed)
{
	if (!attached_ || deaf_ || peers_ < 2)
		return false;

	note(cc, doubleSpeed);
	applyDue();
	return lit_ != 0;
}

void LinkInfrared::apply(const Edge &edge)
{
	unsigned const bit = 1u << (edge.from < 31 ? edge.from : 31);

	if (edge.on)
		lit_ |= bit;
	else
		lit_ &= ~bit;
}

void LinkInfrared::queue(const Edge &edge)
{
	if (edgeCount_ >= EDGE_MAX)
	{
		/* A peer cannot have flashed this often inside one horizon, so this is
		 * the reasoning being wrong rather than the buffer being small. The
		 * oldest edge is acted on early to make room, which keeps the LED's
		 * final state right at the cost of when it got there, and it is said
		 * once. How many messages have arrived by a rendezvous is fixed by the
		 * grants, so even this is the same on two peers replaying it. */
		if (!overflowed_)
		{
			overflowed_ = true;
			gambatte_log(RETRO_LOG_WARN,
				"Infrared: more edges waiting than the horizon should allow;"
				" acting on the oldest early\n");
		}
		apply(edges_[edgeHead_]);
		edgeHead_ = (edgeHead_ + 1) % EDGE_MAX;
		--edgeCount_;
	}
	edges_[(edgeHead_ + edgeCount_) % EDGE_MAX] = edge;
	++edgeCount_;
}

void LinkInfrared::applyDue()
{
	while (edgeCount_ > 0 && edges_[edgeHead_].tick <= now_)
	{
		apply(edges_[edgeHead_]);
		edgeHead_ = (edgeHead_ + 1) % EDGE_MAX;
		--edgeCount_;
	}
}

void LinkInfrared::pump()
{
	unsigned char buf[IR_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(buf);

	if (!attached_)
		return;

	while (link_->recv(handle_, &tick, &from, buf, &len))
	{
		if (len == IR_MSG_SIZE && buf[0] == IR_LEVEL)
		{
			Edge edge;
			edge.tick = tick;
			edge.from = (unsigned char)(from < 255 ? from : 255);
			edge.on = buf[2] & 1;
			/* Not while deaf: a port that waits for nobody is told things
			 * when it is told them, and is not listening anyway. */
			++edgesIn_;
			if (!deaf_ && tick < now_)
			{
				++edgesLate_;
				if (now_ - tick > worstLate_)
					worstLate_ = now_ - tick;
			}
			queue(edge);
		}
		len = sizeof(buf);
	}
}

void LinkInfrared::refreshPeers()
{
	unsigned was = peers_;
	unsigned count = 0;
	int id;

	if (!attached_)
		return;

	id = link_->peers(handle_, &count);
	if (id < 0)
	{
		selfId_ = 0;
		peers_ = 0;
	}
	else
	{
		selfId_ = id;
		peers_ = count;
	}

	if (peers_ != was)
	{
		/* Forget who was lit, and say so again if this machine is. What was in
		 * view a moment ago is not evidence about what is in view now, and a
		 * unit that came into view since has never been told anything. */
		forgetPeers();
		anchored_ = false;
		reannounceUntil_ = now_ + IR_REANNOUNCE;
		gambatte_log(RETRO_LOG_INFO, "Infrared: %u unit(s) in view, this one is %d\n",
		             peers_, selfId_);
	}
}

unsigned LinkInfrared::advance(unsigned wanted)
{
	uint32_t wakeFlags = RETRO_LINK_WAKE_NONE;
	uint64_t safe;
	uint64_t reach;
	uint64_t headroom;
	unsigned budget;

	if (!attached_)
		return wanted;

	refreshPeers();

	horizon_ = hot() ? IR_LATENCY : IR_IDLE_HORIZON;
	safe = now_ + horizon_;
	if (safe < promised_)
		safe = promised_;

	/* Never asks for more than its own horizon, which is what keeps two units
	 * from each waiting for the other to go first: whichever has promised the
	 * least is asking for no more than everybody has already promised it. Nor
	 * for more than the chunk can use, since an idle horizon is longer than
	 * one and asking for all of it would make a peer run that far ahead for
	 * nothing.
	 *
	 * A deaf port asks for nothing at all. That is granted whatever the others
	 * are doing, so it publishes where it is and waits for nobody. */
	reach = (uint64_t)wanted * 4;
	if (reach > horizon_)
		reach = horizon_;
	grant_ = link_->advance(handle_, now_, safe, deaf_ ? 0 : now_ + reach, &wakeFlags);
	promised_ = safe;
	anchored_ = true;

	if (wakeFlags & RETRO_LINK_WAKE_TOPOLOGY)
	{
		/* The bus was rebuilt under this machine. The count may not have
		 * changed -- one unit swapped for another leaves it at two -- so the
		 * count alone cannot be trusted to say so. */
		forgetPeers();
		reannounceUntil_ = now_ + IR_REANNOUNCE;
	}

	pump();
	applyDue();

	/* A flash is an edge and is sent as one, but an LED that is simply ON when
	 * somebody comes into view has no edge left to send. */
	if (led_ && peers_ >= 2 && now_ < reannounceUntil_)
		wire(true);

	if (deaf_ || grant_ == RETRO_LINK_UNBOUNDED)
		return wanted;
	if (wakeFlags != RETRO_LINK_WAKE_NONE && grant_ <= now_)
		return 1;

	headroom = (grant_ > now_) ? (grant_ - now_) : 0;
	budget = (unsigned)(headroom / 4);

	/* Never zero: see LinkSerial::advance. */
	if (budget < 8)
		budget = 8;
	if (budget > wanted)
		budget = wanted;
	return budget;
}
