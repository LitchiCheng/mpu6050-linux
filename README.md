# mpu6050-linux
linux driver for mpu6050

## dts
- 在i2c控制器下追加节点
```c
mpu6050:mpu6050@68 {
    compatible = "dar,mpu6050";
    reg = <0x68>;
    status = "okay";
};
```

## make
- 修改KERNELDIR为内核目录
```c
KERNELDIR := /home/dar/Project/linux/kernal_source/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/
```

## demo
- mpu6050Demo为测试程序，读原始ADC和实际值