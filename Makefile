NAME=kbd-cart-cmd
PREFIX ?= ${HOME}/local/DIR/${NAME}
DEB_TARGET=kbd-cart-cmd-0.1-Linux.deb


default: build

configure:
	mkdir -p build
	cd build && cmake -DCMAKE_INSTALL_PREFIX=${PREFIX} ..

build: configure
	cd build && make

install: build
	cd build && make install

xstow_install: install
	cd ${PREFIX}/../ && xstow ${NAME}

xstow_uninstall:
	cd ${PREFIX}/../ && xstow -D ${NAME} && rm -rf ${NAME}

deb: build
	cd build && make package

deb_install: deb
	cd build && sudo dpkg -i ${DEB_TARGET}

clean:
	rm -rf build/

