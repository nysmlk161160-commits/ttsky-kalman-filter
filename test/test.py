import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer


async def write_value(dut, address, value):
    for high in (0, 1):
        await FallingEdge(dut.clk)
        dut.ui_in.value = (value >> (8 * high)) & 255
        dut.uio_in.value = address | (high << 2) | (1 << 3)
        await FallingEdge(dut.clk)
        dut.uio_in.value = 0


async def read_value(dut, select_p):
    result = 0
    for high in (0, 1):
        dut.uio_in.value = (select_p << 5) | (high << 2)
        await Timer(1, unit="ns")
        result |= int(dut.uo_out.value) << (8 * high)
    return result & 1023


@cocotb.test()
async def kalman_test(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    cases = [
        (96, 0, 64, 320),
        (-96, 0, 64, 320),
        (160, 64, 128, 64),
        (0, 64, 64, 0),
        (32, -64, 32, 32),
        (511, -512, 511, 511),
    ]

    for z, x, p, r in cases:
        await write_value(dut, 0, z & 1023)
        await write_value(dut, 1, x & 1023)
        await write_value(dut, 2, p)
        await write_value(dut, 3, r)

        await FallingEdge(dut.clk)
        dut.uio_in.value = 1 << 4
        await FallingEdge(dut.clk)
        dut.uio_in.value = 0

        for _ in range(80):
            await ClockCycles(dut.clk, 1)
            await Timer(1, unit="ns")
            if int(dut.uio_out.value) & 128:
                break
        else:
            assert False, "Timeout: Kalman core did not finish"

        gain = (p << 6) // (p + r)
        expected_x = (x + ((gain * (z - x)) >> 6)) & 1023
        expected_p = (p - ((gain * p) >> 6)) & 1023
        actual_x = await read_value(dut, 0)
        actual_p = await read_value(dut, 1)
        assert actual_x == expected_x, (z, x, p, r, actual_x, expected_x)
        assert actual_p == expected_p, (z, x, p, r, actual_p, expected_p)
