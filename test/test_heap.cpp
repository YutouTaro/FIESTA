//
// Created by yutou on 3/2/22.
//

#include <bits/stdc++.h>
#include <ESDFMap.h>
#include <cstdlib>
#include <utility>

#include <parameters.h>

using namespace std;
typedef pair<double, int> HeapElement;

//struct struct1{
//    int x;
//    struct1(int a){
//        x = a;
//    }
//    bool operator<(const struct1& struct1_) const{
//        return x < struct1_.x;
//    }
//};
//
//struct cmp{
//    bool operator() (struct1 a, struct1 b){
//        return a.x < b.x;
//    }
//};

struct QueueElement {
    QueueElement(Eigen::Vector3i point, double distance){
        this->point_ = move(point);
        this->distance_ = distance;
        this->squared_distance_ = distance * distance;
    }
    QueueElement(Eigen::Vector3i point, double distance, Eigen::Vector3i closest_obstacle){
        this->point_ = move(point);
        this->distance_ = distance;
        this->closest_obstacle_ = move(closest_obstacle);
        this->squared_distance_ = distance * distance;
    }
    Eigen::Vector3i point_;
    double distance_;
    Eigen::Vector3i closest_obstacle_;
    double squared_distance_;
    bool operator<(const QueueElement &element) const {
        return squared_distance_ > element.squared_distance_;
    }
};

struct cmp_HeapElement{
    bool operator() (HeapElement element1, HeapElement element2){
        return element1.first > element2.first;
    }
};
typedef priority_queue<HeapElement , vector<HeapElement>, cmp_HeapElement> DistanceHeap;

int main(){
//    priority_queue<int> pq1;
//    pq1.push(5);
//    pq1.push(1);
//    pq1.push(10);
//    pq1.push(30);
//    pq1.push(20);
//
//    while (!pq1.empty()){
//        cout << pq1.top() << " ";
//        pq1.pop();
//    }
//    cout << endl;
//
////    priority_queue<struct1> pq2;
//    priority_queue<struct1, vector<struct1>, cmp> pq2;
//    pq2.push(struct1(1));
//    pq2.push(struct1(2));
//    pq2.push(struct1(3));
//    pq2.push(struct1(4));
//    while (!pq2.empty()){
//        cout << pq2.top().x << " ";
//        pq2.pop();
//    }
//    cout << endl;

    vector<QueueElement> queue_;
    DistanceHeap pq;

//    int HI = 10, LO = -10;
//    int range = HI - LO + 1;
    Eigen::Vector3i obstacle(0, 0, 0);
    int N = fiesta::num_dirs_;

    // values to be searched later
    Eigen::Vector3i target_point;
    double target_sqr_dist;

    for (int i=0; i<N; i++){
//        Eigen::Vector3i point = Eigen::Vector3i::Constant(1, 3, 0);
//        for (int ii=0; ii<3; ii++){
//            point[ii] = rand() % range + LO;
//        }
        Eigen::Vector3i point = obstacle + fiesta::dirs_[i];
        QueueElement tmp_element(point, point.cast<float>().norm(), obstacle);
        queue_.push_back(tmp_element);
//        pri_queue.push(tmp_element);
        HeapElement mypair(tmp_element.squared_distance_, queue_.size()-1);
        pq.push(mypair);
        if (i==3){
            target_point = point;
            target_sqr_dist = tmp_element.squared_distance_;
        }
    }

    // print the queue
    cout << "Ordered by inserting order" << endl;
    for (int i=0; i<N; i++){
        cout << "i=" << i << "\t " << queue_[i].point_.x() << "\t " << queue_[i].point_.y() << "\t " << queue_[i].point_.z() << ",\t dist=" << queue_[i].distance_ << ",\t sqr_dist=" << queue_[i].squared_distance_ << endl;
    }

    // look for a voxel by indexing the point coordinate
    vector<QueueElement>::iterator it = find_if(queue_.begin(), queue_.end(), boost::bind(&QueueElement::point_, _1) == target_point);
    if (it != queue_.end()){
        cout << it->distance_ << endl;
    }

    // print the heap
    cout << "Ordered by distance" << endl;
    double sqr_dist = -1;
    vector<QueueElement> processing_batch;
    while (!pq.empty()){
        int queue_idx = pq.top().second;
        sqr_dist = queue_[queue_idx].squared_distance_;
        while (!pq.empty() && abs(sqr_dist - queue_[queue_idx].squared_distance_) < 1e-4){
            cout << "i=" << queue_idx << "\t " << queue_[queue_idx].point_.x() << "\t " << queue_[queue_idx].point_.y() << "\t " << queue_[queue_idx].point_.z() << ",\t dist=" << queue_[queue_idx].distance_ << ",\t sqr_dist=" << queue_[queue_idx].squared_distance_ << endl;
            processing_batch.push_back(queue_[queue_idx]);
            pq.pop();
            queue_idx = pq.top().second;
        }
        cout << processing_batch.size() << " in batch of sqr_distance = " << sqr_dist << endl;
        processing_batch.clear();
    }

    // find an element in heap

    return 0;
}