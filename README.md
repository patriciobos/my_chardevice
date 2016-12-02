# my_chardevice
linux char device driver 

make compila los siguientes fuentes:
  my_chardriver.c
  writer.c
  reader.c

./inst_module removes previously inserted module, clean dmesg log and re-inserts the module

./test_writer script runs "./writer /dev/my_chardevice"

./test_reader script runs "./reader /dev/my_chardevice"
