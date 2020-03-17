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
	bool operator()(const Node_pointer& left, const Node_pointer& right)const  //���ڣ���������
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
};//�Զ���set����׼�����Ȱ�cost���򣬴��Ű�x����x��ķ��ڿ�ʼ��
using Node_set=set<Node_pointer,sort_Node>;

struct obstacle {
	//�ϰ�������
	//�ϰ�����·��Լ��������˫�򷨱�ʾ
	float x_c,y_c,R_c;
	float vx,vy;
	obstacle() = default;
};
using obs_set=set<obstacle*>; 
/*class path_side {
public:
	//·������
	//�ڴ˽�����·��
	float path_l;
	float ;
	float vx, vy;
	obstacle() = default;
};*/

class Astar {
public:
	Astar() = default;
	list<float>path_x, path_y;
	list<Node>search_Node;//�洢���������������ݣ�������Ϊָ������
	bool check_crash(point* new_point, obs_set obs_list, Node* node_father);
	float caculate_cost_g(point* new_point, point* end_point, Node* node_father);
	float caculate_cost_h(point* new_point, point* end_point, Node* node_father);
	void update_vertices(Node_set& openlist_, set<Node_pointer>& closelist_, Node* new_node, point* end_point);
	void find_path_node(point* start_point, point* end_point, float(*search_derection)[2], obs_set& obs);
	const float*get_dis_len() {return &(this->dis_len);}
	const float*get_R_car() { return &(this->R_car); }
	float dis_len = 0.5;//�����ڳ�βԲ�ĵ���ͷԲ�ľ���
	float R_car = 1;//����Բ�İ뾶
private:

};

//����յ㵽�յ�֮������rrt��node
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
	list<Node_rrt>search_Node_rrt;//�洢һ�������ڵ�
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