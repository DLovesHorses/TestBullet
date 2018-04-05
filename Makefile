LIB_BULLET=./lib/bullet

BULLET_LIBS = ${LIB_BULLET}

SOURCES = main.cpp

CFLAGS= -std=c++11 -I${LIB_BULLET}/src
LDFLAGS=-L${LIB_BULLET}/bin/src/BulletDynamics -L${LIB_BULLET}/bin/src/BulletCollision -L${LIB_BULLET}/bin/src/LinearMath -lBulletDynamics -lBulletCollision -lLinearMath

make_dirs:
	mkdir ./bin -p

all: make_dirs
	g++ ${CFLAGS} ${SOURCES} ${LDFLAGS} -o ./bin/testbullet.o
