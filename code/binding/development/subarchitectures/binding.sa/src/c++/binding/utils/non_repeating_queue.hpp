#ifndef NON_REPEATING_QUEUE_HPP_
#define NON_REPEATING_QUEUE_HPP_


namespace Binding {

/// uses std::set and std::deque to implement a simple queue with no
/// repetitions
template <typename T>
class non_repeating_queue {
  std::deque<T> m_queue;
  std::set<T> m_set;
public:
  /// puts the object on the queue and returns true if the entry was
  /// not already there
  bool enqueue(const T& _t) {
    if(m_set.find(_t) == m_set.end()) {
      m_set.insert(_t);
      m_queue.push_back(_t);
      return true;
    }
    return false;
  }
  /// gets the first entry on the queue and removes it from the queue
  T dequeue() {
    assert(!this->empty());
    T t(m_queue.front());
    m_queue.pop_front();
    m_set.erase(t);
    return t;
  }
  bool empty() const {
    return m_queue.empty();
  }
  typedef typename std::deque<T>::const_iterator const_iterator;
  const_iterator begin() const {return m_queue.begin();}
  const_iterator end() const {return m_queue.end();}
};
} // namespace Binding

#endif
