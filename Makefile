INCLUDES = -I lib/arturoemx/ -I lib/arturoemx/Circle/ -I src/tools/ -I src/kf/ -I src/ball_tracking/
CXXFLAGS = -Wall -g $(INCLUDES)
CIRCLEDIR = lib/arturoemx/Circle

all: kalman_filter_ball_tracking

$(CIRCLEDIR)/objs/Circle.o: $(CIRCLEDIR)/Circle.cpp
	(cd $(CIRCLEDIR); make)

kalman_filter_ball_tracking: objs/kalman_filter_ball_tracking.o $(CIRCLEDIR)/objs/Circle.o
	g++ $(CXXFLAGS) -o kalman_filter_ball_tracking objs/kalman_filter_ball_tracking.o `pkg-config opencv4 --libs` $(CIRCLEDIR)/objs/Circle.o

objs/kalman_filter_ball_tracking.o: kalman_filter_ball_tracking.cpp
	g++ $(CXXFLAGS) -o objs/kalman_filter_ball_tracking.o -c kalman_filter_ball_tracking.cpp `pkg-config opencv4 --cflags`

clean:
	rm objs/*.o kalman_filter_ball_tracking
