#include<set>
#include<vector>
#include<list>
#include <algorithm>
using namespace std;
struct point
{
	float x, y;
	bool operator==(const point& coor_xy) {
		return((x == coor_xy.x) & (y == coor_xy.y));
	}
};
struct Node
{
	point coordinate;
	float g, h;
	Node* father;
	Node() = default;
	Node(point coordinate_, Node* father_ = nullptr) {
		coordinate = coordinate_;
		father = father_;
		g = 0;
		h = 100000;
	}
	float cost() {
		return g + h;
	};
};
using Node_pointer=Node *;
struct sort_Node {
	bool operator()(const Node_pointer& left, const Node_pointer& right)const  //对于（）的重载
	{
		if (left->cost() < right->cost()) {return true;}
		else if (left->cost() == right->cost()) {
			if (left->coordinate.x > right->coordinate.x) { return true; }
			else if (left->coordinate.x == right->coordinate.x) {
				if (left->coordinate.y > right->coordinate.y) { return true; }
				else { return false; }
			}
			else { return false; }
		}
		else { return false; }
		//return (left->cost() < right->cost());
	}
};//自定义set排序准则；优先按cost排序，次优按x排序x大的放于开始处
using Node_set=set<Node_pointer,sort_Node>;

struct obstacle {
	//障碍物数量
	//障碍车与路边约束均采用双球法表示
	float x_c,y_c,R_c;
	float vx,vy;
	obstacle() = default;
};
using obs_set=set<obstacle*>; 
/*class path_side {
public:
	//路边数据
	//在此仅考虑路宽
	float path_l;
	float ;
	float vx, vy;
	obstacle() = default;
};*/

class Astar {
public:
	Astar() = default;
	list<float>path_x, path_y;
	list<Node>search_Node;//存储所有搜索过的数据，其他均为指针运算
	bool check_crash(point* new_point, obs_set obs_list, Node* node_father);
	float caculate_cost_g(point* new_point, point* end_point, Node* node_father);
	float caculate_cost_h(point* new_point, point* end_point, Node* node_father);
	void update_vertices(Node_set& openlist_, set<Node_pointer>& closelist_, Node* new_node, point* end_point);
	void find_path_node(point* start_point, point* end_point, float(*search_derection)[2], obs_set& obs);
	const float*get_dis_len() {return &(this->dis_len);}
	const float*get_R_car() { return &(this->R_car); }
	float dis_len = 0.5;//车辆第车尾圆心到车头圆心距离
	float R_car = 1;//车辆圆心半径
private:

};

//定义拐点到拐点之间撒点rrt的node
struct Node_rrt {
	float x, y, s_path, heading;
	Node_rrt* father;
	//Node_rrt() = default;
	Node_rrt(Node_rrt* father_ = nullptr) {
		father = father_;
		x = 0;
		y = 0;
		s_path = 0;
		heading = 0;
		//heading = (father == nullptr ? 0 : (y - father->y) / (x - father->x));
		//s_path = (father == nullptr ? 0 : father->s_path + (float)pow(((double)x - father->x) * ((double)x - father->x) + ((double)y - father->y) * ((double)y - father->y), 0.5));
	}
};
class non_cons_path :public Astar {
public:
	non_cons_path() = default;
	list<float*> break_point_x, break_point_y;
	vector<float>path_x_sm, path_y_sm;
	list<Node_rrt>search_Node_rrt;//存储一段搜索节点
	void find_break_point_data();
	bool check_kapa_cons(Node_rrt*current);
	void generate_Node_rand(Node_rrt* end_node, Node_rrt* rand_node);
	bool find_father_node_rrt(Node_rrt* rand_node);
	void caculate_state(Node_rrt* son, Node_rrt* father);
	bool compare_dis_rrt(list<Node_rrt>::iterator& first, list<Node_rrt>::iterator& secend, Node_rrt* rand_);
	bool check_arrive_rrt(Node_rrt* current, Node_rrt* end_node);
	bool check_crash_rrt(Node_rrt* current, obs_set& obs_list, Node_rrt* current_father);
	void find_path_rrt(set<obstacle*>& obs);
private:
	float deta_s = 0.5f;
	float p = 60;
	float map_size[4];
};