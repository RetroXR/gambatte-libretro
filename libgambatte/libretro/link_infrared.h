#ifndef _LINK_INFRARED_H
#define _LINK_INFRARED_H

#include <gambatte.h>
#include <stdint.h>

#include "link_interface.h"

/* A Game Boy Color's infrared port carried by the frontend's link bus.
 *
 * The same bus the link cable rides, on a port of its own, because it is a
 * different thing joining different machines: a lead goes where it is pushed
 * in, and light goes wherever the top of the handheld is pointing. What the
 * port is "cabled" to is the frontend's decision here as there -- two units
 * facing each other is its to judge -- and with nothing in view the bus grants
 * without bound and the receiver sees the dark it always saw.
 *
 * WHAT CROSSES. One message: this machine's LED changed, and when. There is no
 * byte, no clock and no handshake to carry, because the hardware has none --
 * RP is a lamp and an eye, and every protocol over it is the game flashing the
 * one and timing the other against its own timer. Pokemon tells a 0 from a 1 by
 * whether the gap between two flashes is 183 or 366 microseconds, so the one
 * thing this must not do is move two edges by different amounts.
 *
 * Which is why every edge is stamped the same distance ahead. The delay is the
 * commit horizon the bus needs anyway, and a flash arriving a tenth of a
 * millisecond late is still exactly as long as it was sent.
 *
 * HOW MANY. Light is not a cable and nothing limits it to a pair: every unit on
 * the bus sees every other, and the receiver reads lit while any of them is.
 *
 * WHAT DOES NOT. The receiver's own behaviour -- it warms up, and it stops
 * seeing a light that stays on -- is not modelled, nor is a unit seeing its own
 * LED. Neither is anything that is not another Game Boy Color: a television
 * remote, a Pocket Pikachu, or the port a HuC1 or HuC3 cartridge carries.
 */
class LinkInfrared : public gambatte::InfraredIO
{
	public:
		LinkInfrared();
		virtual ~LinkInfrared();

		/* Remember the bus. Called once, from retro_init. */
		void setInterface(const struct retro_link_interface *link, unsigned port);
		bool available() const { return link_ != 0; }

		bool start();
		void stop();
		bool started() const { return attached_; }

		/* Whether this machine waits for the others on its infrared bus.
		 *
		 * It must not while its link cable is carrying a peer. A machine that
		 * waits on two buses can be waited on through both, and the two buses
		 * line their machines up separately: the cable can want this one to hold
		 * for the other while the light wants the other to hold for this one, and
		 * nothing in either would ever move again. So a machine waits on ONE, and
		 * the cable is the one, because a transfer over it cannot be a little
		 * late.
		 *
		 * A deaf port still says where it is, so nobody waits on it, and still
		 * flashes -- sending needs no waiting. It only stops seeing. */
		void setDeaf(bool deaf) { deaf_ = deaf; }

		/* Called from retro_run before each chunk of emulation, with how many
		 * stereo samples it means to produce. Returns how many it may, which is
		 * `wanted` unchanged when nothing is in view. */
		unsigned advance(unsigned wanted);

		/* And after each chunk, with what it actually produced. See
		 * LinkSerial::produced for why samples are the measure. */
		void produced(unsigned samples);

		unsigned peers() const { return peers_; }

		/* InfraredIO. */
		virtual void begin(unsigned long cc, bool doubleSpeed);
		virtual void write(unsigned long cc, bool doubleSpeed, unsigned char rp);
		virtual bool lit(unsigned long cc, bool doubleSpeed);
		virtual void restored(unsigned char rp);

	private:
		/* A peer's LED changing, held until this machine's clock reaches the
		 * tick it was stamped with. WHEN the message arrived is a wall-clock
		 * accident; when its tick comes round is not. */
		struct Edge
		{
			uint64_t tick;
			unsigned char from;
			unsigned char on;
		};

		/* Room for a peer toggling its LED as fast as a Game Boy can write a
		 * register, for as long as an idle horizon lasts. */
		enum { EDGE_MAX = 1024 };

		void note(unsigned long cc, bool doubleSpeed);
		void setLed(bool on);
		void wire(bool on);
		void pump();
		void queue(const Edge &edge);
		void apply(const Edge &edge);
		void applyDue();
		void refreshPeers();
		void forgetPeers();
		bool hot() const;

		const struct retro_link_interface *link_;
		unsigned port_;
		retro_link_port_t *handle_;
		bool attached_;
		bool deaf_;

		int selfId_;
		unsigned peers_;

		/* This machine's position on the link timeline, in ticks of a clock
		 * running at twice the Game Boy's normal 4.194304 MHz -- see
		 * LinkSerial::now_ for why twice. Read off the cycle counter inside a
		 * chunk, from the reading begin() takes at the start of it, and put back
		 * on the frontend's own sample count at the end. */
		uint64_t now_;
		uint64_t floor_;
		unsigned long lastRaw_;
		bool haveRaw_;

		/* How far ahead this machine promises not to flash, and the furthest
		 * tick it has promised that for.
		 *
		 * Short while the port is in use and long while it is not, because a
		 * horizon is paid for in rendezvous and two handhelds left facing each
		 * other on a shelf should cost nearly nothing. The promise outlives the
		 * horizon that made it: one given while the port was idle still binds
		 * the first flash after it wakes. */
		uint64_t horizon_;
		uint64_t promised_;
		uint64_t grant_;
		bool anchored_;
		uint64_t reannounceUntil_;

		/* RP as last written, and the LED out of it. */
		unsigned char rp_;
		bool led_;

		/* How far ahead an edge is stamped, and the edges that decide it.
		 *
		 * The latency, unless a promise made while idle reaches further -- and
		 * then it STAYS further for the rest of that burst, because the next
		 * edge has to be moved by exactly as much as this one or the flash
		 * between them changes length. It goes back only across a silence no
		 * protocol times. */
		uint64_t delay_;
		uint64_t lastEdge_;
		uint64_t lastStamp_;
		bool edgeSeen_;

		/* Which peers' LEDs are lit, one bit a peer, as of this machine's clock. */
		unsigned lit_;

		Edge edges_[EDGE_MAX];
		unsigned edgeHead_;
		unsigned edgeCount_;
		bool overflowed_;

		/* How many edges have come in, how many of them after the tick they
		 * were stamped for, and the worst of those. An edge should only ever
		 * be late by the few ticks a chunk overshoots its grant. Counted
		 * always, since it is one comparison an edge, and said once as the
		 * port leaves the bus if GBC_IR_STATS is set in the environment --
		 * which is how to find out on a machine that cannot be watched any
		 * other way. Read by nothing that decides anything. */
		unsigned edgesIn_;
		unsigned edgesLate_;
		uint64_t worstLate_;
};

#endif
