#include <bits/stdc++.h>
#ifndef INDEXED_PRIORITY_QUEUE_H_

#define INDEXED_PRIORITY_QUEUE_H_
using namespace std;

namespace fiesta {
    template<class T1, class T2,
            class Comparator=less <T2>,
            class Hash = hash <T1> >
    class IndexedPriorityQueue {

        typedef vector <pair<T1, T2>> ContainerType;

        // Storing indices of values using key
        unordered_map<T1, long long, Hash> m;
        // Container
        ContainerType v;
        // Size
        long long numElement;
        // instance of Comparator class
        Comparator comp;
        // Max Capacity
        long long CAPACITY_ = LLONG_MAX;

        long long get_parent_index(long long index) {
            if (index == 0) {
//            cout << "root node does not have parent node" << endl;
                return -1;
            }
            return (index - 1) / 2;
        }

        long long get_left_child(long long index) {
            long long left_child = 2 * index + 1;
            if (left_child < numElement) {
                return left_child;
            } else {
                return -1;
            }
        }

        long long get_right_child(long long index) {
            long long right_child = 2 * index + 2;
            if (right_child < numElement) {
                return right_child;
            } else {
                return -1;
            }
        }

        // get index value from hash map
        long long getValueIndex(T1 key) {
            if (m[key] == 0) {
                cout << "The key does not exist! " << key << endl;
                return -1;
            }
            return v[m[key] - 1];
        }

        void heapify_up(long long index = -1) {
            if (index == -1) {
                index = numElement - 1;
            }

            pair <T1, T2> element = v[index];
            long long index_parent = get_parent_index(index);
            bool element_swapped = false;
            while (index_parent >= 0 &&
                   comp(v[index_parent].second, element.second)) {
                element_swapped = true;
                // swap
                v[index] = v[index_parent];
                // update map
                m[v[index].first] = index + 1;

                index = index_parent;
                index_parent = get_parent_index(index);
            }
            if (element_swapped) {
                v[index] = element;
                m[v[index].first] = index + 1;
            }
        }

        void heapify_down(long long index = 0) {
            pair <T1, T2> element = v[index];
            long long suitableIndex = index; // the final index that the element going to be placed
            long long leftChild = get_left_child(suitableIndex),
                    rightChild = get_right_child(suitableIndex);

            while (leftChild > 0) { // at least left child exists
                // find the node with the larger value in parent, leftchild and rightchild
                long long largerChild;
                if (rightChild > 0 && comp(v[leftChild].second, v[rightChild].second)) {
                    // right child exists and larger than left child
                    largerChild = rightChild;
                } else {
                    largerChild = leftChild;
                }
                // compare with the element
                if (comp(element.second, v[largerChild].second)) {
                    // swap
                    v[suitableIndex] = v[largerChild];
                    // update map
                    m[v[suitableIndex].first] = suitableIndex + 1;
                } else { // the element is larger than its two children
                    break;
                }
                suitableIndex = largerChild;
                leftChild = get_left_child(suitableIndex);
                rightChild = get_right_child(suitableIndex);
            }
            if (suitableIndex != index) {
                v[suitableIndex] = element;
                m[v[suitableIndex].first] = suitableIndex + 1;
            }
        }

    public:
        IndexedPriorityQueue() {
            numElement = 0;
            m.clear();
            v.clear();
        }

        void push(T1 key, T2 value) {
            if (numElement >= CAPACITY_) {
                cout << "The indexed priority queue is overflowed!";
                return;
            }
            if (m[key] != 0) {
                // replace the value
                changeAtKey(key, value);
                return; // heap is done in changeAtKey() function
            }

            // Adding element
            pair <T1, T2> new_element = make_pair(key, value);
            v.push_back(new_element);
            numElement++;
            m[key] = numElement;
            heapify_up(-1);
        }

        void pop() {
            if (numElement == 0) {
                cout << "No element to pop" << endl;
                return;
            }

            // remove root node
            // remove the key in m[]
            m.erase(v[0].first);
            // replace the root node by the last node
            if (numElement > 1) {
                v[0] = v[numElement - 1];
                m[v[0].first] = 1; // note that in this line, now v[0] == v[-1]
            }
            v.erase(v.end());
            numElement--;
            // heap down
            heapify_down(0);
        }

        pair <T1, T2> top() {
            return v[0];
        }

        long long size() {
            return numElement;
        }

        bool empty() {
            return numElement == 0;
        }

        void changeAtKey(T1 key, T2 value) {
            if (m[key] == 0) {
                cout << "No element match key " << key << endl;
                return;
            }
            long long index = m[key] - 1;

            if (comp(value, v[index].second)) {
                v[index].second = value;
                heapify_down(index);
            } else {
                v[index].second = value;
                heapify_up(index);
            }
        }

        void print_v() {
            cout << "v: ";
            for (auto &vi: v) {
                cout << "(" << vi.first << ", " << vi.second << ") ";
            }
            cout << endl;
        }

        void print_m() {
            cout << "m: ";
            for (auto &mi: m) {
                cout << "(" << mi.first << ", " << mi.second << ") ";
            }
            cout << endl;
        }
    };

    // Type of queue element to be used in priority queue
    struct IPQ_element {
        IPQ_element(Eigen::Vector3i point, double distance) {
            this->point_ = std::move(point);
            this->distance_ = distance;
            this->squared_distance_ = distance * distance;
        }

        IPQ_element(Eigen::Vector3i point, double distance, Eigen::Vector3i closest_obstacle) {
            this->point_ = std::move(point);
            this->distance_ = distance;
            this->closest_obstacle_ = std::move(closest_obstacle);
            this->squared_distance_ = distance * distance;
        }

        Eigen::Vector3i point_;
        double distance_;
        Eigen::Vector3i closest_obstacle_;
        double squared_distance_;

        bool operator<(const IPQ_element &element) const {
            return squared_distance_ > element.squared_distance_;
        }
    };

    typedef std::pair<Eigen::Vector3i/*point*/, double/*sqr_dist*/> HeapElement; //typedef std::vector<HeapElement>::iterator HeapIter;
    struct cmp_element {
        bool operator()(IPQ_element element1, IPQ_element element2) {
            return element1.squared_distance_ > element2.squared_distance_;
        }
    };

//    struct cmp_HeapElement{
//        bool operator() (HeapElement element1, HeapElement element2){
//            return element1.first > element2.first;
//        }
//    };
//    typedef std::priority_queue<HeapElement , std::vector<HeapElement>, cmp_HeapElement> DistanceHeap;
    typedef std::priority_queue<IPQ_element, std::vector<IPQ_element>, cmp_element> DistanceHeap;
}

#endif // INDEXED_PRIORITY_QUEUE_H_

//void display(const IndexedPriorityQueue<int, int> &IPQ) {
//    IndexedPriorityQueue<int, int> temp = IPQ;
//    while (!temp.empty()) {
//        pair<int, int> tmp = temp.top();
//        temp.pop();
//        cout << "(" << tmp.first << ", " << tmp.second << ")\n";
//    }
//}

//int main(int argc, char** argv){
//    IndexedPriorityQueue<int, int> IPQ;
//
//    // Check if empty
//    cout << "Checking if initially the IPQ is empty\n";
//    if (IPQ.empty())
//        cout << "IPQ is empty\n";
//    else
//        cout << "IPQ is not empty\n";
//    cout << endl;
//
//    vector<int> A = {2,7,26,25,19,17,1,90,3,36};
//    // Insertion
//    cout << "Insert\n";
////    IPQ.push(2, 1);
////    IPQ.push(3, 7);
////    IPQ.push(1, 0);
////    IPQ.push(4, 5);
//    for (int i=0; i<A.size(); i++){
//        IPQ.push(i, A[i]);
//    }
//
//    // Printing the contents of IPQ
//    cout << "IPQ: \n";
//    display(IPQ);
//    cout << endl;
//
//    // Checking size and top after pushing
//    cout << "Size: " << IPQ.size() << endl;
//    cout << "Top: " << IPQ.top().first
//         << ", " << IPQ.top().second
//         << "\n\n";
//
//    // Replace operation
//    cout << "Changing value" << endl;
////    IPQ.changeAtKey(9, 18);
////    display(IPQ);
//    IPQ.changeAtKey(0, 40);
//    display(IPQ);
//
////    // Checking size and top after replacement
////    cout << "Size: " << IPQ.size() << endl;
////    cout << "Top: " << IPQ.top().first
////         << ", " << IPQ.top().second
////         << "\n\n";
////
////    // Deleting 2 elements from IPQ
////    cout << "Poping an element from IPQ: ";
////    IPQ.pop();
////    cout << "\nPoping an element from IPQ: ";
////    IPQ.pop();
////    cout << endl << endl;
////
////    // Printing the contents of IPQ after deletion
////    cout << "IPQ: \n";
////    display(IPQ);
////    cout << '\n';
////
////    // Checking size and top after pushing
////    cout << "Size: " << IPQ.size() << endl;
////    cout << "Top: " << IPQ.top().first
////         << ", " << IPQ.top().second
////         << "\n\n";
//
//    return 0;
//}