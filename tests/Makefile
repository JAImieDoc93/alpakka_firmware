gyro_filter_test: gyro_filter_test.c ../src/gyro_filter.c ../src/headers/gyro_filter.h
	cc -O2 -Wall -Wextra -I../src/headers -o $@ gyro_filter_test.c ../src/gyro_filter.c -lm

run: gyro_filter_test
	./gyro_filter_test > results.md; cat results.md
