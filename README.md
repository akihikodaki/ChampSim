<p align="center">
  <h1 align="center"> Tyche-Artifact </h1>
  <p>  <p>
</p>

## What's Tyche?

[Tyche](https://dl.acm.org/doi/10.1145/3641853) is a hardware prefetcher designed to improve the performance of indirect memory access(IMAs) patterns. It utilizes a bilateral propagation mechanism to accurately uncover the instruction dependencies in simple chains with moderate length (rather than complex graphs). Based on the exact instruction dependencies, Tyche can effectively recognize a variety of IMAs patterns, including nonlinear ones, and generate accurate prefetching requests continuously.

## About The Framework

Tyche is integrated within the [ChampSim](https://github.com/ChampSim/ChampSim) simulator. Since the ChampSim does not maintain the operand values, decoding information, or the execution unit, we have made several modifications. These modifications entail:

1. expanding the original Champsim trace format to encompass both the instruction code and execution result. 
2. introducing two structures, the register file and the memory, to maintain the architectural-level state during execution. 
3. incorporating decode logic and execution unit to traverse instructions in the dependency chain.


## Compile

ChampSim takes a JSON configuration script. Examine `champsim_config.json` for a fully-specified example. All options described in this file are optional and will be replaced with defaults if not specified. The configuration scrip can also be run without input, in which case an empty file is assumed.
```
$ ./config.sh <configuration file>
$ make
```

## Run simulation

Execute the binary directly.
```
$ bin/champsim --warmup_instructions 20000000 --simulation_instructions 100000000 -loongarch trace_name.champsim.trace.xz
```

If you encounter "mmap error!", please execute the command "echo 1 > /proc/sys/vm/overcommit_memory".

## Trace

The traces are generated by [QEMU](https://github.com/qemu/qemu), which has been customized for the purpose of trace generation. The QEMU plugin for trace generation can be found [here](https://github.com/rrwhx/qemu_plugins_loongarch); thanks to [Xinyu Li](https://github.com/rrwhx) for providing this plugin. Once the plugin is installed, you can generate traces for LoongArch. For reference, an illustrative trace sample (a segment of hj2) and its binary executable file are available at [here](https://drive.google.com/drive/folders/1Xw_jWKLdUpdNekc5cNrVGP4k2mLRf5_I?usp=sharing).

## Cite

If you find our work useful, please consider citing:

```
@article{xue2024tyche,
  title={Tyche: An Efficient and General Prefetcher for Indirect Memory Accesses},
  author={Xue, Feng and Han, Chenji and Li, Xinyu and Wu, Junliang and Zhang, Tingting and Liu, Tianyi and Hao, Yifan and Du, Zidong and Guo, Qi and Zhang, Fuxin},
  journal={ACM Transactions on Architecture and Code Optimization},
  year={2024},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  doi = {10.1145/3641853}
}
```