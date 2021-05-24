## Ultra-Wideband Swarm Ranging
This repository contains the implementation of the paper:  
**Ultra-Wideband Swarm Ranging**. [Feng Shan](http://twinhorse.net/), Jiaxin Zeng, Zengbao Li, [Junzhou Luo](https://cse.seu.edu.cn/2019/0102/c23024a257045/page.htm), [Weiwei Wu](https://cse.seu.edu.cn/2019/0103/c23024a257230/page.htm). *INFOCOM 2021*. [PDF](http://twinhorse.net/papers/SZLLW-INFOCOM21p.pdf),  
which proposed a UWB-based ranging protocol for dynamic and dense swarm of robots and devices.

> This repository is forked from the [official firmware by Bitcraze](https://github.com/bitcraze/crazyflie-firmware) and all our work is stored in the folder `crazyflie-firmware/src/deck/drivers/src/swarming`.  
> If you find any error, please don't hesitate to open an issue or pull request.

If you find this repository helpful for your work, please kindly cite the following paper. The Bibtex is listed below:

```
@article{shan2021ultra,
  title={Ultra-Wideband Swarm Ranging},
  author={Shan, Feng and Zeng, Jiaxin and Li, Zengbao and Luo, Junzhou and Wu, Weiwei},
  booktitle={IEEE INFOCOM 2021-IEEE Conference on Computer Communications},
  year={2021},
  organization={IEEE}
}
```

## How to run it

Clone this repository:

## Building and Flashing
See the [building and flashing instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) in the github docs folder.

Enter the `swarming` folder:

```
cd crazyflie-firmware/src/deck/drivers/src/swarming
```

Build the firmware:

```
make clean
make
```

Flash the **cf2.bin**:

```
cfloader flash path/to/cf2.bin stm32-fw
```

If you have any question, please issue the project or [email](mailto:shanfeng@seu.edu.cn) us and we will reply you soon.
