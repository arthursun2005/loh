all:
	g++ -Ofast -mavx2 -std=gnu++20 -Wall -Ieigen *.cc -o loh -lsfml-graphics -lsfml-window -lsfml-system -lfmt -lpthread
install:
	install ./loh /usr/local/bin
