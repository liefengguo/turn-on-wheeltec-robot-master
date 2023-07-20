#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <deque>
#include <iostream>
#include <vector>
class AdaptiveFilter {
public:
    AdaptiveFilter(int bufferSize, int threshold);
    int filter(int value);
    std::vector<int32_t> filter(std::vector<int32_t> value);

private:
    int bufferSize;
    int threshold;
    std::deque<int> buffer;
    int average;


    bool isOutlier(int value) const;
    int calculateAverage() const;
    int calculateStandardDeviation() const;
};
#endif  // ADAPTIVE_FILTER_H
