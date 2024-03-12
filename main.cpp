#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

struct Robot
{
    int x, y, goods; // 是否携带物品
    int status;      // 0表示恢复，1表示正常运行
    int mbx, mby;    // 此帧机器人所在的位置
    Robot() {}
    Robot(int startX, int startY)
    {
        x = startX;
        y = startY;
    }
} robot[robot_num + 10];

struct Berth
{
    int x;
    int y;
    int transport_time;
    int loading_speed;
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
} berth[berth_num + 10];

struct Boat
{
    // 船的容积是一样的
    int num;    // 携带物品的价值
    int pos;    // 目标泊位，-1表示虚拟点
    int status; // 0表示移动（从地图上消失），1表示装货状态或运输完成，2表示等待
} boat[10];

int money, boat_capacity, id;
char ch[N][N]; // 字符地图
int gds[N][N];
void Init()
{
    for (int i = 1; i <= n; i++)
        scanf("%s", ch[i] + 1);
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

int Input()
{
    scanf("%d%d", &id, &money);
    int num; // 新增货物的数量
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int x, y, val; // 货物的坐标和价值
        scanf("%d%d%d", &x, &y, &val);
        // 把货物存起来........
    }
    for (int i = 0; i < robot_num; i++)
    {
        int sts;
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
        // 根据状态处理机器人........
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    // 处理船只........
    char okk[100];
    scanf("%s", okk);
    return id;
}

int main()
{
    Init();
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        int id = Input();
        for (int i = 0; i < robot_num; i++)
            // 为机器人下达命令........
            printf("move %d %d\n", i, rand() % 4);
        // 为船只下达命令........
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
