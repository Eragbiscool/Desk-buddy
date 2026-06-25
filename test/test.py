"""
cocotb testbench -- tt_um_luck_engine v2

All tests drive only ui_in and observe uo_out (the documented status flags)
plus, for digit content, the real TM1637 serial bus via TM1637Monitor --
decoding actual wire bytes back to digit values, the same way a real
TM1637 module (or a logic analyzer) would see them. No internal
hierarchical signal probing.

Note for EDA Playground users
------------------------------
The TM1637Monitor class is defined directly in this file (rather than
imported from a separate tm1637_monitor.py) because EDA Playground's
cocotb environment does not automatically add the test directory to
Python's module search path, causing "name cocotb is not defined" errors
when a separate-file import fails silently and leaves the module in a
broken half-imported state. Everything needed to run the full suite is
self-contained in this single file.
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


# ─── TM1637 bus monitor (inlined) ────────────────────────────────────────────
class TM1637Monitor:
    """
    Passive TM1637 bus monitor: watches the real CLK/DIO pins (uio_out /
    uio_oe / uio_in) and decodes start/stop/byte events the way a real
    TM1637 slave or logic analyzer would see them -- verifying actual wire
    traffic rather than trusting the RTL's own internal state registers.

    Also acts as a minimal simulated slave: whenever the chip releases DIO
    (uio_oe[1]==0) to listen for ACK, this monitor drives uio_in[1] low,
    the same as a real TM1637 pulling the line low to acknowledge.
    """
    def __init__(self, dut):
        self.dut = dut
        self.bytes_seen = []   # (txn_index, byte_value) in chronological order
        self.starts = 0
        self.stops = 0

    def _clk(self):
        return int(self.dut.uio_out.value) & 1

    def _dio_oe(self):
        return (int(self.dut.uio_oe.value) >> 1) & 1

    def _dio(self):
        if self._dio_oe():
            return (int(self.dut.uio_out.value) >> 1) & 1
        return 0  # simulated slave always ACKs (pulls low) when released

    async def run(self):
        # Wait until reset has been driven and released before touching any
        # pin -- at time 0, before do_reset() runs, uio_out/uio_oe are X
        # (unknown), and int()-ing an X value raises in cocotb.
        while True:
            await Timer(1, units="ns")
            try:
                if int(self.dut.rst_n.value) == 1:
                    break
            except ValueError:
                continue

        prev_clk = self._clk()
        prev_dio = self._dio()
        bit_buf = []
        txn_index = -1

        while True:
            await Timer(1, units="ns")

            # drive the simulated ACK back into the DUT's input
            if self._dio_oe() == 0:
                cur = int(self.dut.uio_in.value)
                self.dut.uio_in.value = (cur & ~0b10) | (0 << 1)

            cur_clk = self._clk()
            cur_dio = self._dio()

            # start condition: clk steady high, dio falls
            if cur_clk == 1 and prev_clk == 1 and prev_dio == 1 and cur_dio == 0:
                self.starts += 1
                bit_buf = []
                txn_index += 1
            # stop condition: clk steady high, dio rises
            elif cur_clk == 1 and prev_clk == 1 and prev_dio == 0 and cur_dio == 1:
                self.stops += 1

            # rising clk edge: sample a data bit (first 8) or the ACK slot (9th)
            if cur_clk == 1 and prev_clk == 0:
                if len(bit_buf) < 8:
                    bit_buf.append(cur_dio)
                    if len(bit_buf) == 8:
                        val = 0
                        for i, b in enumerate(bit_buf):
                            val |= (b << i)
                        self.bytes_seen.append((txn_index, val))
                else:
                    bit_buf = []  # ACK slot consumed; next bit starts a new byte

            prev_clk = cur_clk
            prev_dio = cur_dio

    def start(self):
        cocotb.start_soon(self.run())

PRESCALE = 10
DEBOUNCE = 4
CLICK_WINDOW = 8

MODE_COIN, MODE_BUSY, MODE_SPRINT, MODE_MEETING, MODE_TASK, MODE_DUEL = range(6)

BTN_GO, BTN_TAP, BTN_P2, BTN_RESET = 3, 4, 5, 6

# segment patterns -> digit code, reverse of tm1637_driver's seg_of()
SEG_TO_CODE = {
    0x3F: 0, 0x06: 1, 0x5B: 2, 0x4F: 3, 0x66: 4, 0x6D: 5, 0x7D: 6,
    0x07: 7, 0x7F: 8, 0x6F: 9, 0x00: "blank", 0x40: "dash",
    0x39: "C", 0x3D: "G", 0x50: "r", 0x53: "?",
}


class Stimulus:
    """Write-only shadow of ui_in -- never reads the signal back, avoiding
    the write-then-immediate-readback race documented in the v1 testbench."""
    def __init__(self, dut):
        self.dut = dut
        self.value = 0
        self.dut.ui_in.value = 0

    def _push(self):
        self.dut.ui_in.value = self.value

    def set_mode(self, mode):
        self.value = (self.value & ~0b111) | (mode & 0b111)
        self._push()

    def set_bit(self, bit, level):
        if level: self.value |=  (1 << bit)
        else:     self.value &= ~(1 << bit)
        self._push()

    async def press(self, bit, hold=DEBOUNCE + 3, release=DEBOUNCE + 3):
        self.set_bit(bit, 1)
        await ClockCycles(self.dut.clk, hold)
        self.set_bit(bit, 0)
        await ClockCycles(self.dut.clk, release)

    async def double_click(self, bit):
        """Two clearly-separate debounced presses, spaced well inside the
        click-classifier's window. hold=7 matches the margin already proven
        reliable elsewhere in this suite; release=4 (=DEBOUNCE) gives the
        synchronizer enough low time to fully reset between presses -- a
        shorter release was found to merge both presses into a single
        detected edge instead of two."""
        await self.press(bit, hold=DEBOUNCE + 3, release=DEBOUNCE)
        await self.press(bit, hold=DEBOUNCE + 3, release=DEBOUNCE)


def flags(dut):
    u = int(dut.uo_out.value)
    return {
        "active":     u & 1,
        "p1_ready":   (u >> 1) & 1,
        "p2_ready":   (u >> 2) & 1,
        "warn":       (u >> 3) & 1,
        "timer_run":  (u >> 4) & 1,
        "alarm":      (u >> 5) & 1,
        "busy_light": (u >> 6) & 1,
        "penalty":    (u >> 7) & 1,
    }


async def do_reset(dut):
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.ui_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return Stimulus(dut)


def start_clock_and_monitor(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    mon = TM1637Monitor(dut)
    mon.start()
    return mon


def last_full_refresh_digits(mon):
    """Find the most recent complete 7-byte refresh group in mon.bytes_seen
    and return (d0,d1,d2,d3) decoded segment->code values, or None if no
    complete refresh has happened yet. Bytes are addr-cmd-skipped: each
    refresh is 3 transactions (txn boundaries by construction): [CMD1],
    [CMD2,d0,d1,d2,d3], [CMD3]. We identify the 5-byte middle transaction
    by length and decode bytes 1..4 of it (byte 0 is the CMD2 address byte)."""
    by_txn = {}
    for txn_idx, val in mon.bytes_seen:
        by_txn.setdefault(txn_idx, []).append(val)
    for txn_idx in sorted(by_txn.keys(), reverse=True):
        b = by_txn[txn_idx]
        if len(b) == 5 and b[0] == 0xC0:
            digits = [SEG_TO_CODE.get(v, f"?{v:#04x}") for v in b[1:]]
            return tuple(digits)
    return None


# ─────────────────────────────────────────────────────────────────────────
@cocotb.test()
async def test_reset_state(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_COIN)
    await ClockCycles(dut.clk, 2)
    f = flags(dut)
    assert f["active"] == 0 and f["alarm"] == 0 and f["busy_light"] == 0


@cocotb.test()
async def test_glitch_rejected(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_COIN)
    await ClockCycles(dut.clk, 2)
    stim.set_bit(BTN_GO, 1)
    await ClockCycles(dut.clk, DEBOUNCE - 2)
    stim.set_bit(BTN_GO, 0)
    await ClockCycles(dut.clk, 20)
    assert flags(dut)["active"] == 0, "sub-debounce glitch should not register"


@cocotb.test()
async def test_coin_flip_bus_verified(dut):
    """Verify the coin result over the REAL TM1637 bus, not internal state."""
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_COIN)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_GO)
    await ClockCycles(dut.clk, 60 + 700)  # roll animation + a full TM1637 refresh

    assert mon.starts > 0 and mon.stops > 0, "no TM1637 bus activity observed"
    digits = last_full_refresh_digits(mon)
    assert digits is not None, "no complete 5-byte digit refresh captured"
    d0, d1, d2, d3 = digits
    assert d3 in (0, 1), f"coin result digit should be 0 or 1, decoded {d3}"
    assert flags(dut)["active"] == 1


@cocotb.test()
async def test_task_roulette_bus_verified(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_TASK)
    await ClockCycles(dut.clk, 2)
    for _ in range(5):
        await stim.press(BTN_TAP)
    await stim.press(BTN_GO)
    await ClockCycles(dut.clk, 700)

    digits = last_full_refresh_digits(mon)
    assert digits is not None
    d3 = digits[3]
    assert isinstance(d3, int) and 1 <= d3 <= 5, f"task result must be 1-5, decoded {d3}"


@cocotb.test()
async def test_meeting_warn_and_alarm(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_MEETING)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_TAP)   # 1 tap = 5 min
    await stim.press(BTN_GO)
    await ClockCycles(dut.clk, 5)
    assert flags(dut)["timer_run"] == 1

    await ClockCycles(dut.clk, 5 * 60 * PRESCALE + 20)
    f = flags(dut)
    assert f["alarm"] == 1, "alarm should fire once the meeting timer reaches zero"
    assert f["timer_run"] == 0


@cocotb.test()
async def test_sprint_clock(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_SPRINT)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_GO)
    await ClockCycles(dut.clk, 3 * PRESCALE + 5)  # SHOW phase
    assert flags(dut)["timer_run"] == 1

    await ClockCycles(dut.clk, 31 * 60 * PRESCALE + 50)
    assert flags(dut)["alarm"] == 1


@cocotb.test()
async def test_busy_mode_lifecycle(dut):
    """Tap once -> 30 min busy timer starts, busy_light + timer_run assert,
    and the TM1637 bus shows '30' on the last two digits."""
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_BUSY)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_TAP)
    await ClockCycles(dut.clk, 700)

    f = flags(dut)
    assert f["busy_light"] == 1 and f["timer_run"] == 1
    digits = last_full_refresh_digits(mon)
    assert digits is not None
    d2, d3 = digits[2], digits[3]
    assert d2 == 3 and d3 == 0, f"expected '30' on the display, decoded d2={d2} d3={d3}"


@cocotb.test()
async def test_busy_double_click_qmark_and_resume(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_BUSY)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_TAP)
    await ClockCycles(dut.clk, 50)

    await stim.double_click(BTN_TAP)
    await ClockCycles(dut.clk, 700)
    digits = last_full_refresh_digits(mon)
    assert digits is not None and digits[3] == "?", f"expected '?' glyph, got {digits}"
    assert flags(dut)["busy_light"] == 1, "busy_light should stay on while paused-display"
    assert flags(dut)["timer_run"] == 1, "timer must keep running in the background"

    # single click resumes -> display should show a real number again
    await stim.press(BTN_TAP)
    await ClockCycles(dut.clk, 700)
    digits2 = last_full_refresh_digits(mon)
    assert isinstance(digits2[3], int), f"expected a real digit after resume, got {digits2}"


@cocotb.test()
async def test_busy_double_click_cancel(dut):
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_BUSY)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_TAP)
    await ClockCycles(dut.clk, 20)

    await stim.double_click(BTN_TAP)   # -> QMARK
    await ClockCycles(dut.clk, 20)
    await stim.double_click(BTN_TAP)   # -> cancel
    await ClockCycles(dut.clk, 5)

    f = flags(dut)
    assert f["busy_light"] == 0, "busy_light should turn off after the cancel double-click"
    assert f["active"] == 0


@cocotb.test()
async def test_duel_clean_race_and_penalty(dut):
    """Race to G; whichever player presses first (after G) wins cleanly.
    Verified via the real TM1637 bus, not internal signals."""
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_DUEL)
    await ClockCycles(dut.clk, 2)
    assert flags(dut)["p1_ready"] == 0 and flags(dut)["p2_ready"] == 0

    await stim.press(BTN_TAP)
    assert flags(dut)["p1_ready"] == 1
    await stim.press(BTN_P2)
    assert flags(dut)["p2_ready"] == 1

    # Wait through the full possible C delay (2-10s) watching for 'G' on the bus
    saw_g = False
    for _ in range(900):  # max C delay (~100 cycles) + ~2 TM1637 refresh periods margin
        await RisingEdge(dut.clk)
        d = last_full_refresh_digits(mon)
        if d is not None and d[3] == "G":
            saw_g = True
            break
    assert saw_g, "duel should reach the G (go) state within the max delay window"

    # Player 1 presses first -> should win cleanly, no penalty
    await stim.press(BTN_TAP, hold=DEBOUNCE + 1, release=DEBOUNCE + 1)
    await ClockCycles(dut.clk, 700)
    f = flags(dut)
    assert f["alarm"] == 1
    assert f["penalty"] == 0, "a clean post-G press should never be flagged as a penalty"
    digits = last_full_refresh_digits(mon)
    assert digits[3] == 1, f"player 1 pressed first and should win, decoded {digits}"


@cocotb.test()
async def test_duel_false_start_penalty(dut):
    """Pressing during the C (countdown) phase is a false start: the OTHER
    player wins by penalty, flagged distinctly from a clean race win."""
    mon = start_clock_and_monitor(dut)
    stim = await do_reset(dut)
    stim.set_mode(MODE_DUEL)
    await ClockCycles(dut.clk, 2)
    await stim.press(BTN_TAP)
    await stim.press(BTN_P2)
    # now in C; press p1 immediately as a false start
    await stim.press(BTN_TAP, hold=DEBOUNCE + 1, release=DEBOUNCE + 1)
    await ClockCycles(dut.clk, 700)

    f = flags(dut)
    assert f["alarm"] == 1
    assert f["penalty"] == 1, "an early press during C must be flagged as a penalty win"
    digits = last_full_refresh_digits(mon)
    assert digits[3] == 2, f"player 1 false-started, player 2 should win by penalty, decoded {digits}"
