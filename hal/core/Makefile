objs := main.o bus.o spi.o init.o

main: ${objs}
	gcc -o main ${objs} -Wl,-Telf.lds

${objs}:%.o:%.c
	cc -c -o $@ $< -g

clean:
	-rm main *.o

