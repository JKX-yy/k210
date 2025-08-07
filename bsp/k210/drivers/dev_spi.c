#include <rtdevice.h>
#include <dev_spi.h>
/*
**********************自己写的   在component中已经有了**********

*/
//创建一个spi设备  挂在spi总线上     片选 引脚
rt_err_t  rt_hw_spi_device_attach(const char *bus_name, const char *device_name,int cs_index, int cs_pin)
{
    struct  rt_spi_device *spi_device;
    struct rt_spi_bus *spi_bus;
    struct rt_spi_cs *cs;
    RT_ASSERT(bus_name);
    RT_ASSERT(device_name);
    
    spi_bus=(struct rt_spi_bus *)rt_device_find(bus_name);
    if(spi_bus == RT_NULL)
    {
        rt_kprintf("SPI bus %s not found!\n", bus_name);
        return -RT_ENOSYS;
    }
    cs=rt_malloc(sizeof(struct rt_spi_cs));
    if(!cs)
    {
        rt_kprintf("Can't allocate memory for SPI device %s!\n", device_name);
        return -RT_ENOMEM;
    }
    cs->cs_index = cs_index; //设置片选索引
    cs->cs_pin = cs_pin; //设置片选引脚
    rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);
    rt_pin_write(cs_pin, PIN_HIGH); //设置片选引脚为高电平（未选中状态）

    spi_device=rt_malloc(sizeof(struct rt_spi_device));
    if(!spi_device)
    {
        rt_kprintf("Can't allocate memory for SPI device %s!\n", device_name);
        rt_free(cs);
        return -RT_ENOMEM;
    }
    spi_device->parent.type = RT_Device_Class_SPIDevice;
    spi_device->bus = spi_bus;
    spi_device->cs_pin = cs;
    rt_device_register(&spi_device->parent, device_name, RT_DEVICE_FLAG_RDWR);
    
    
    return rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs);



}