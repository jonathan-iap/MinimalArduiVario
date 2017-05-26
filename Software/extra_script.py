Import('env')
env.Replace(FUSESCMD="avrdude $UPLOADERFLAGS -e -Uhfuse:w:0xDA:m -Uefuse:w:0xFE:m -Ulfuse:w:0xE2:m")
