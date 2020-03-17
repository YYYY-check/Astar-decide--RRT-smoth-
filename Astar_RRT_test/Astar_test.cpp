﻿#include <iostream>
#include"Astar.h"
using namespace std;
int main()
{
	obs_set Obslist;
	obstacle Obs;
	Obs.x_c = 20, Obs.y_c = 0, Obs.R_c = 5, Obs.vx = 0, Obs.vy = 0;
	Obslist.insert(&Obs);
	point start_ = { 0,0 };
	point end_ = { 40,0 };
	non_cons_path Path_;
	float derection[8][2] = { { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },{ -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } };
	Path_.find_path_node(&start_, &end_, derection, Obslist);
	list<float>::iterator i, j;
	i = Path_.path_x.begin();
	j = Path_.path_y.begin();
	/*while (i != Path_.path_x.end()) {
		cout << *i << "," << *j << endl;
		i++;
		j++;
	}*/
	Path_.find_break_point_data();
	Path_.find_path_rrt(Obslist);
	vector<float>::iterator m, n;
	m = Path_.path_x_sm.begin();
	n = Path_.path_y_sm.begin();
	while (m != Path_.path_x_sm.end()) {
		cout << *m << "," << *n << endl;
		m++;
		n++;
	}
	//cout << start_.x << "," << start_.y << endl;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
