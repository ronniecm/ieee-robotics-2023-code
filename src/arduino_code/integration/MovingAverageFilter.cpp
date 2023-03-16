#include "MovingAverageFilter.hpp"

MovingAverageFilter::MovingAverageFilter(int numReadings)
  : _numReadings(numReadings), _readIndex(0), _total(0) {
  _readings = new int[_numReadings];
  for (int i = 0; i < _numReadings; i++) {
    _readings[i] = 0;
  }
}

MovingAverageFilter::~MovingAverageFilter() {
  delete[] _readings;
}

int MovingAverageFilter::process(int newReading) {
  _total -= _readings[_readIndex];
  _readings[_readIndex] = newReading;
  _total += _readings[_readIndex];
  _readIndex = (_readIndex + 1) % _numReadings;
  return _total / _numReadings;
}
