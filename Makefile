DESTDIR=/usr/local

ocotp: ocotp.c

install:
	install -d $(DESTDIR)/bin
	install ocotp $(DESTDIR)/bin

clean:
	rm -rf ocotp *.o
