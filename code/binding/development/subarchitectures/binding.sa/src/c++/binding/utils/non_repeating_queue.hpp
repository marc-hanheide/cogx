#ifndef NON_REPEATING_QUEUE_HPP_
#define NON_REPEATING_QUEUE_HPP_


namespace Binding {

/// uses std::set and std::deque to implement a simple queue with no
/// repetitions
template <typename T>
class non_repeating_queue {
  std::deque<T> queue;
  std::set<T> set;
public:
  /// puts the object on the queue and returns true if the entry was
  /// not already there
  bool enqueue(const T& _t) {
    if(set.find(_t) == set.end()) {
      set.insert(_t);
      queue.push_back(_t);
      return true;
    }
    return false;
  }
  /// gets the first entry on the queue and removes it from the queue
  T dequeue() {
    assert(!this->empty());
    T t(queue.front());
    queue.pop_front();
    set.erase(t);
    return t;
  }
  bool empty() const {
    return queue.empty();
  }
  typedef typename std::deque<T>::const_iterator const_iterator;
  const_iterator begin() const {return queue.begin();}
  const_iterator end() const {return queue.end();}
};
} // namespace Binding

#endif
