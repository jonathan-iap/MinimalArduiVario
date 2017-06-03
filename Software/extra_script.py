Import('env')
env.Replace(FUSESCMD="avrdude $UPLOADERFLAGS -e -Uhfuse:w:0xda:m -Uefuse:w:0xff:m -Ulfuse:w:0xe2:m")
