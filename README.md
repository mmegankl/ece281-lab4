# Lab 4: Advanced Elevator Controller

VHDL for ECE 281 [Lab 4](https://usafa-ece.github.io/ece281-book/lab/lab4.html)

Targeted toward Digilent Basys3. Make sure to install the [board files](https://github.com/Xilinx/XilinxBoardStore/tree/2018.2/boards/Digilent/basys3).

Tested on Windows 11.

---

## Build the project

You can simply open the `.xpr` and Vivado will do the rest!

## GitHub Actions Testbench

The workflow uses the [setup-ghdl-ci](https://github.com/ghdl/setup-ghdl-ci) GitHub action
to run a *nightly* build of [GHDL](https://ghdl.github.io/ghdl/).

The workflow uses GHDL to analyze, elaborate, and run the entity specified in the `.github/workflows/testbench.yml`.

```yaml
env:
  TESTBENCH_ENTITY: myfile
```

If successful then GHDL will quietly exit with a `0` code.
If any of the `assert` statements fail **with** `severity failure` then GHDL will cease the simulation and exit with non-zero code; this will also cause the workflow to fail.
Assert statements of other severity levels, such as "error" w

Documentation Statement: I collaborated with C3C Cook and C3C Morales as lab partners to complete the code in Vivado. All three of us attended a video conference EI session on 16Apr with Lt Col Trimble to fix any errors and complete the advanced functionality aspect of the lab. 