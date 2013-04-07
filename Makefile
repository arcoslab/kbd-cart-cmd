default: kbd-cart-cmd

CFLAGS = -g
#CFLAGS = -O3


kbd-cart-cmd: kbd-cart-cmd.cc
	g++ ${CFLAGS}   -lpthread -L${HOME}/local/lib -lYARP_OS -lYARP_init -o kbd-cart-cmd kbd-cart-cmd.cc
clean:
	rm -f kbd-cart-cmd

