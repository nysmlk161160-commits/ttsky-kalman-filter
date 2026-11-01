## How it works

The project performs one scalar Kalman measurement update using Q4.6 fixed-point arithmetic:

- K = P / (P + R)
- x_out = x + K * (z - x)
- P_out = P - K * P

An iterative divider computes the scaled Kalman gain and a shared seven-cycle serial multiplier computes two products. An FSM controls the arithmetic and registers the outputs.

## How to test

The `test/test.py` Cocotb test applies multiple signed and unsigned input cases.

Use `ui_in[7:0]` for the input byte and `uio_in[1:0]` for the register address (0=z, 1=x, 2=P, 3=R). Set `uio_in[2]` to 0 for the low byte or 1 for the upper two bits. Assert `uio_in[3]` for a clock cycle to write. Assert `uio_in[4]` for one clock cycle to start. Wait for `uio_out[7]` (ready). `uio_in[5]` selects x_out (0) or P_out (1); `uio_in[2]` selects which byte is returned on `uo_out`.

Example: z=1.5, x=0, P=1, R=5 (Q4.6 codes 96, 0, 64, 320). Expected result codes: x_out=15 and P_out=54.

The inputs are synchronous for this study design. Use P+R > 0 and avoid output overflow. The 50 MHz value in info.yaml is a target, not a verified clock rate; run STA and gate-level checks.

## External hardware

None required for simulation.
