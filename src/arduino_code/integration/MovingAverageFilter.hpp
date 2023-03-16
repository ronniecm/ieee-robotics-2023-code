#ifndef MOVING_AVERAGE_FILTER_HPP
#define MOVING_AVERAGE_FILTER_HPP

class MovingAverageFilter {
  public:
    MovingAverageFilter(int numReadings);
    ~MovingAverageFilter();
    int process(int newReading);

  private:
    int _numReadings;
    int *_readings;
    int _readIndex;
    int _total;
};

#endif // MOVING_AVERAGE_FILTER_HPP
