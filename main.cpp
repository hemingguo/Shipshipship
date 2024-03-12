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
    bool is_occupied; // 如果有船占用或有船驶向此泊位，则为true
    queue<int> goods; // 待装载的货物
    Berth()
    {
        is_occupied = false;
    }
    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
        is_occupied = false;
    }
} berth[berth_num + 10];

// 船的specific_status
#define WAIT 3
#define LOAD 4
#define DONE 5
#define TO_BERTH 6
#define TO_VIRTUAL 7

struct Boat
{
    // 船的容积是一样的
    int num;             // 船上物品的数量
    int goods_value;     // 船上物品的价值
    int pos;             // 目标泊位，-1表示虚拟点
    int status;          // 0表示移动（从地图上消失），1表示装货状态或运输完成，2表示等待
    int specific_status; // WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点
    Boat()
    {
        num = 0;
        goods_value = 0;
        specific_status = DONE;
    }
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
/**
 * \brief 维护船只的具体状态，
 * WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点；
 * 以及船只目前装载的货物的数量和价值，泊位的货物队列
 */
void UpdateBoatSpecificState(int boat_id)
{
    if (boat[boat_id].status == 1)
    {
        if (boat[boat_id].specific_status == TO_BERTH || boat[boat_id].specific_status == WAIT)
        {
            boat[boat_id].specific_status = LOAD;
        }
        else if (boat[boat_id].specific_status == TO_VIRTUAL)
        {
            boat[boat_id].specific_status = DONE;
        }
    }
    else if (boat[boat_id].status == 2)
    {
        boat[boat_id].specific_status = WAIT;
    }
    if (boat[boat_id].specific_status == LOAD)
    {
        for (int i = 0; i < berth[boat[boat_id].pos].loading_speed; i++)
        {
            if (berth[boat[boat_id].pos].goods.empty())
                break;
            boat[boat_id].goods_value += berth[boat[boat_id].pos].goods.front();
            berth[boat[boat_id].pos].goods.pop();
            boat[boat_id].num += 1;
        }
    }
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
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    // 维护船只的具体状态........
    for (int i = 0; i < 5; i++)
        UpdateBoatSpecificState(i);
    char okk[100];
    scanf("%s", okk);
    return id;
}

/**
 * \brief 计算目前泊位上船只可装下的货物的总价值
 */
int GetTotalValue(int berth_id)
{
    int total_value = 0; // 能装载的货物的总价值
    int cnt = 0;         // 目前装载的货物的数量
    while (cnt < boat_capacity && cnt < berth[berth_id].goods.size())
    {
        if (berth[berth_id].goods.empty())
            break;
        int tmp = berth[berth_id].goods.front();
        total_value += tmp;
        berth[berth_id].goods.pop();
        berth[berth_id].goods.push(tmp);
        cnt++;
    }
    return total_value;
}

// 超参数
int total_value_weight = 100;
int transport_time_weight = 10;
int is_occupied_weight = 1000;
int loading_speed_weight = 1;

/**
 * \brief 需要考虑如何计算泊位的优先级......
 */
int GetBerthPriority(int berth_id)
{
    int priority = 0;
    int total_value = GetTotalValue(berth_id);
    // 计算泊位的优先级
    if (berth[berth_id].is_occupied)
    {
        priority = total_value * total_value_weight - berth[berth_id].transport_time * transport_time_weight - is_occupied_weight - berth[berth_id].loading_speed * loading_speed_weight;
    }
    else
    {
        priority = total_value * total_value_weight - berth[berth_id].transport_time * transport_time_weight - berth[berth_id].loading_speed * loading_speed_weight;
    }
    return priority;
}

/**
 * \brief 获取船只的目标泊位
 */
int GetBerthId()
{
    int priority[berth_num + 10]; // 计算泊位的优先级，数值越大优先级越高
    int tmp_priority;
    for (int i = 0; i < berth_num; i++)
    {
        tmp_priority = GetBerthPriority(i);
        priority[i] = tmp_priority;
    }
    // 选择优先级最高的泊位
    int max_priority = -99999;
    int max_priority_id = -1;
    for (int i = 0; i < berth_num; i++)
    {
        if (priority[i] > max_priority)
        {
            max_priority = priority[i];
            max_priority_id = i;
        }
    }
    berth[max_priority_id].is_occupied = true;
    return max_priority_id;
}

/**
 * \brief 装了多少货物后，船只离开泊位进行运输，需要考虑怎么设置阈值.......
 */
int GetThreshold(int boat_id)
{
    int threshold = berth[boat[boat_id].pos].transport_time;
    return threshold;
}

bool BoatReadyGo(int boat_id)
{
    int threshold = GetThreshold(boat_id);
    // 船只满载或者价值超过阈值
    if (boat[boat_id].num >= boat_capacity || boat[boat_id].goods_value >= threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * \brief 只考虑了在虚拟点时需下达命令，在泊位装货时需下达命令.......
 */
void GiveBoatCommand()
{
    for (int i = 0; i < 5; i++)
    {
        if (boat[i].specific_status == DONE)
        { // 如果船只运输完成，在虚拟点
            boat[i].specific_status = TO_BERTH;
            printf("ship %d %d\n", i, GetBerthId());
            continue;
        }
        if (boat[i].specific_status == LOAD)
        {
            // 如果船只在泊位装货
            if (BoatReadyGo(i))
            {
                boat[i].specific_status = TO_VIRTUAL;
                printf("go %d\n", i);
            }
            continue;
        }
    }
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
        void GiveBoatCommand();
        puts("OK");
        fflush(stdout);
    }
    return 0;
}
