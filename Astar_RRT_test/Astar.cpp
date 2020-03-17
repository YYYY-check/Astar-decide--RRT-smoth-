#include"Astar.h"
#include<math.h>
#include <iostream>
#include<ctime>
#define random(x,y) rand()%(int)(y-x)+x
using namespace std;
bool Astar::check_crash(point* new_point, obs_set obs_list,Node* node_father) {
	//����������Ϊһ��Բ�����Բģ�ͺ�������
	bool res = true;
	for (auto obs : obs_list) {
		if ((abs(new_point->x - obs->x_c) >= obs->R_c + R_car) || (abs(new_point->y - obs->y_c) >= obs->R_c + R_car)) continue;
		else if (pow(new_point->x - obs->x_c, 2) + pow(new_point->y - obs->y_c, 2) >= pow(R_car + obs->R_c, 2)) continue;
		else {
			res = false;
			break;
		}
	}
	return res;
}
float Astar::caculate_cost_g(point* new_point, point* end_point,Node*node_father) {
	return node_father->g+pow(pow(new_point->x - node_father->coordinate.x, 2) + pow(new_point->y - node_father->coordinate.y, 2), 0.5);
		//abs(new_point->x - node_father->coordinate.x) + abs(new_point->y - node_father->coordinate.y);
}
float Astar::caculate_cost_h(point* new_point, point* end_point, Node* node_father) {
	return abs(new_point->x - end_point->x) + abs(new_point->y - end_point->y);
}
void Astar::update_vertices(Node_set&openlist_, set<Node_pointer>&closelist_, Node*new_node,point*end_point) {
	bool res_close = true;
	bool res_open = true;
	float G_new=0, H_new=0;
	for (auto list : closelist_) {
		if(list->coordinate== new_node->coordinate){ //������closelist�еĵ���ȥ
			res_close = false;
			break;
		}
	}
	if (res_close) {
		for (Node_set::iterator it = openlist_.begin(); it != openlist_.end();it++) {
			if ((*it)->coordinate == new_node->coordinate) {
				if (new_node->cost() < (*it)->cost()) {//������openlist����ʧ��ԭ��С����ԭ�и��ڵ�
					openlist_.erase(it);
					break;
				}
				else { res_open = false; }
			}
		}
		if (res_open) {
			openlist_.insert(new_node);
		}
	}
};
void Astar::find_path_node(point* start_point, point* end_point, float(*search_derection)[2], obs_set& obs) {
	Node Node_best(*start_point);//�洢ÿ����ʧ��С�Ľڵ�
	point Pnew;//�洢ÿ��������point
	Node Nonew;//�洢ÿ������node
	Node* No_father=nullptr;
	search_Node.push_back(Node_best);
	Node_set openlist;//����cost��С��������
	set<Node*>closelist;//����ָ���С�������򷽱���ݵ���ʼ��
	openlist.insert(&search_Node.back());
	while (!openlist.empty()) {
		Node_best = *(*openlist.begin());
		closelist.insert(*openlist.begin());
		No_father = *openlist.begin();
		if (Node_best.coordinate == *end_point) { break; }
		openlist.erase(openlist.begin());//����ѽڵ����closelist��ɾ��openlist��Ӧ�Ľڵ�
		for (int i = 0; i < 8; i++) {
			Pnew.x = Node_best.coordinate.x + search_derection[i][0];
			Pnew.y = Node_best.coordinate.y + search_derection[i][1];
			Nonew.coordinate = Pnew, Nonew.father = No_father;
			Nonew.g = caculate_cost_g(&Pnew, end_point, Nonew.father);
			Nonew.h = caculate_cost_h(&Pnew, end_point, Nonew.father);
			if (check_crash(&Pnew, obs, Nonew.father)) {
				search_Node.push_back(Nonew);
				update_vertices(openlist, closelist,&search_Node.back(),end_point);
			}
		}
		if (Nonew.father == *openlist.begin()) {
			break;
		}
	}
	//set<Node_pointer>::iterator it=closelist.find(No_father);//
	Node*it = No_father;
	do{
		path_x.push_back(it->coordinate.x);
		path_y.push_back(it->coordinate.y);
		it = it->father;
		//it = find(closelist.begin(),closelist.end(), (*it)->father);
	} while (it->father!=nullptr); 
	path_x.push_back(start_point->x);
	path_y.push_back(start_point->y);
	search_Node.clear();
	openlist.clear();
	closelist.clear();
}
void non_cons_path::find_break_point_data() {
	list<float>::iterator ix, tem_x;
	ix = path_x.end(), ix--;
	list<float>::iterator iy, tem_y;
	iy = path_y.end(), iy--;
	break_point_x.push_back(&(*ix));//��ʼ�����
	break_point_y.push_back(&(*iy));
	--ix, --iy;
	float pn[2], pn_1[2];
	//double in_angle;
	while (distance(path_x.begin(),ix)!=1) {
		tem_x = ix, tem_y = iy;
		pn_1[0] = *ix - *(--tem_x);
		pn[0] = *(++(++tem_x)) - *ix;
		pn_1[1] = *iy - *(--tem_y);
		pn[1] = *(++(++tem_y)) - *iy;
		if ((pn_1[0] * pn[1] - pn_1[1] * pn[0]) != 0) {
			break_point_x.push_back(&(*ix));
			break_point_y.push_back(&(*iy));
		}
		--ix, --iy;
	}
	--ix, --iy;
	break_point_x.push_back(&(*ix));
	break_point_y.push_back(&(*iy));
}
void non_cons_path::caculate_state(Node_rrt* son, Node_rrt* father) {
	son->heading = (father == nullptr ? 0 : (son->y - father->y) / (son->x - father->x));
	son->s_path = (father == nullptr ? 0 : father->s_path + (float)pow(((double)son->x - father->x) * ((double)son->x - father->x) + ((double)son->y - father->y) * ((double)son->y - father->y), 0.5));
}
bool non_cons_path::check_kapa_cons(Node_rrt*current) {
	caculate_state(current, current->father);
	if (abs((current->heading - current->father->heading) / (current->s_path - current->father->s_path)) < 0.7) {
		return true;
	}
	else { return false; }
}
void non_cons_path::generate_Node_rand(Node_rrt* end_node, Node_rrt* rand_node) {
	if (rand() % 100 < p) {
		rand_node->x = end_node->x;
		rand_node->y = end_node->y;
	}
	else {
		rand_node->x = random(map_size[0], map_size[1]);
		rand_node->y = random(map_size[2], map_size[3]);//��Astar����ϵ����һ��
	}
}
bool non_cons_path::find_father_node_rrt(Node_rrt* rand_node) {
	list<Node_rrt>::iterator it = search_Node_rrt.begin();
	list<Node_rrt>::iterator tem;
	tem = it;
	it++;
	for (; it != search_Node_rrt.end(); ++it) {
		if (compare_dis_rrt(tem, it, rand_node)) {
			tem = it;
		}
	}
	float dis_ = pow(pow(tem->x - rand_node->x, 2) + pow(tem->y - rand_node->y, 2), 0.5);
	if (dis_ > deta_s) {
		rand_node->father = &(*tem);
		rand_node->x = tem->x + (-tem->x + rand_node->x) * deta_s / dis_;
		rand_node->y = tem->y + (-tem->y + rand_node->y) * deta_s / dis_;
		return true;
	}
	else if (dis_ == 0) {
		return false;
	}
	else {
		rand_node->father = &(*tem);
		return true;
	}
}
bool non_cons_path::compare_dis_rrt(list<Node_rrt>::iterator& first, list<Node_rrt>::iterator& secend, Node_rrt *rand_) {
	if (abs(first->x - rand_->x) + abs(first->y - rand_->y) > abs(secend->x - rand_->x) + abs(secend->y - rand_->y)) {
		return true;
	}
	else { return false; }
}
bool non_cons_path::check_arrive_rrt(Node_rrt* current, Node_rrt* end_node) {
	if (abs(current->x - end_node->x) < 1 && abs(current->y - end_node->y) < 0.5) { return true; }
	else { return false; }
}
bool non_cons_path::check_crash_rrt(Node_rrt* current, obs_set& obs_list, Node_rrt* current_father) {
	bool res = true;
	for (auto obs : obs_list) {
		if ((abs(current->x - obs->x_c) >= obs->R_c + R_car) || (abs(current->y - obs->y_c) >= obs->R_c + R_car)) continue;
		else if (pow(current->x - obs->x_c, 2) + pow(current->y - obs->y_c, 2) >= pow(R_car + obs->R_c, 2)) continue;
		else {
			res = false;
			break;
		}
	}
	return res;
}
void non_cons_path::find_path_rrt(set<obstacle*>& obs) {
	srand((int)time(0));//�������������
	list<float*>::iterator it_x = break_point_x.begin();
	list<float*>::iterator it_y = break_point_y.begin();
	Node_rrt* _find; 
	Node_rrt first_start(nullptr);
	Node_rrt start_, end_;
	start_.x = *break_point_x.front();
	start_.y = *break_point_y.front();
	Node_rrt*father_first=nullptr;
	Node_rrt rand_;
	vector<float>_x, _y;
	while (distance(it_x,break_point_x.end())>0) {
		start_.father = father_first;
		++it_x, ++it_y;
		++it_x, ++it_y;
		if (distance(it_x, break_point_x.end()) >0) {
			end_.x = *(*it_x), end_.y = *(*it_y);
		}
		else {
			end_.x = *break_point_x.back(), end_.y = *break_point_y.back();
		}
		map_size[0] = min(start_.x, end_.x), map_size[1] = max(start_.x, end_.x);
		map_size[2] = min(start_.y, end_.y) - 4, map_size[3] = max(start_.y, end_.y) + 4;
		search_Node_rrt.push_back(start_);
		int i = 0;
		for (; i < 5000; i++) {
			generate_Node_rand(&end_, &rand_);
			if (find_father_node_rrt(&rand_)) {
				if (check_crash_rrt(&rand_, obs, rand_.father) && check_kapa_cons(&rand_)) {
					search_Node_rrt.push_back(rand_);
				}
				if (check_arrive_rrt(&rand_, &end_)) { break; }
			}
		}
		if (i == 5000) { cout << "Ѱ�ҹյ�֮�����·��ʧ��" << endl; break; }
		//_x.push_back(search_Node_rrt.back().x);
		//_y.push_back(search_Node_rrt.back().y);
		_find = search_Node_rrt.back().father;
		do {
			_x.insert(_x.begin(), _find->x);
			_y.insert(_y.begin(), _find->y);
			_find = _find->father;
		} while (_find != father_first);
		path_x_sm.insert(path_x_sm.end(), _x.begin(), _x.end());
		path_y_sm.insert(path_y_sm.end(), _y.begin(), _y.end());
		_x.clear();
		_y.clear();
		start_.x = search_Node_rrt.back().x, start_.y = search_Node_rrt.back().y;
		start_.heading = search_Node_rrt.back().heading; start_.s_path = search_Node_rrt.back().s_path;
		first_start.x = search_Node_rrt.back().father->x, first_start.y = search_Node_rrt.back().father->y;
		first_start.heading = search_Node_rrt.back().father->heading, first_start.s_path = search_Node_rrt.back().father->s_path;
		father_first = &first_start;
		search_Node_rrt.clear();
	}
	path_x.clear();
	path_y.clear();
}