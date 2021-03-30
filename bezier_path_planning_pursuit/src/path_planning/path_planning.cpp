// ex1.cpp

#include "path_planning.h"
#include <ros/ros.h>

Path::Path(std::string filename)
{
    set_point_csv(filename);
}

void Path::load_config(std::string filename, float accel, float vel, float acc_lim_theta_, float max_vel_theta_, float init_vel, float speed_rate, float path_granularity_)
{
    max_accel = accel;
    max_vel = vel;
    max_initial_speed = init_vel;
    corner_speed_rate = speed_rate;
    path_granularity = path_granularity_;
    set_point_csv(filename);
}

void Path::set_point_csv(std::string filename)
{
    using namespace std;
    string s;
    int count = 0;
    int theta_count = 0;
    ifstream infile;

    // check pnum
    infile.open(filename);
    if (!infile)
    {
        cerr << "err Path::set_point_csv / can not open file" << endl;
        exit(1);
    }
    while (getline(infile, s))
    {
        count++;
    }
    pnum = count - 1;
    if (pnum < 2)
    {
        cerr << "err Path::set_point_csv / csv data error" << std::endl;
        exit(1);
    }
    point.change_size(pnum, 4);
    control_point.change_size(2 * pnum - 2, 2);
    count = 0;
    infile.close();
    // read file
    infile.open(filename);
    while (getline(infile, s))
    {
        if (count == 0)
        {
            count++;
            continue;
        }
        stringstream ss;
        ss << s;
        string sx, sy, stheta;
        getline(ss, sx, ',');
        point[count][1] = atof(sx.c_str());
        getline(ss, sy, ',');
        point[count][2] = atof(sy.c_str());
        getline(ss, stheta, ',');
        if (stheta == "-")
        {
            theta_count++;
        }
        else
        {
            if (count == 1)
                ;
            else
            {
                float diff = (atof(stheta.c_str()) * PI / 180.0f - point[count - theta_count - 1][3]) / (float)(theta_count + 1);
                for (int i = 0; i < theta_count; i++)
                {
                    point[count - theta_count + i][3] = point[count - theta_count + i - 1][3] + diff;
                }
            }
            point[count][3] = atof(stheta.c_str()) * PI / 180.0f;
            theta_count = 0;
        }
        count++;
    }
    infile.close();
    // create path
    bezier();
    set_vel();
}

void Path::set_vel()
{
    using namespace std;
    int waypoint_num_sum = 0;
    waypoint_num = new int[pnum - 1];
    for (int p = 1; p < pnum; p++){
        float length = point[p][4];
        waypoint_num[p - 1] = ceil(length / 1000.0f / path_granularity); // kiriage
        waypoint_num_sum += waypoint_num[p - 1];
    }

    target_vel = new float[waypoint_num_sum + 1];

    int init_index = 0;
    for (int p = 1; p < pnum; p++){
        //P0
        float p0_x = point[p][1] / 1000.0f;
        float p0_y = point[p][2] / 1000.0f;
        //P1
        float p1_x = control_point[2 * p - 1][1] / 1000.0f;
        float p1_y = control_point[2 * p - 1][2] / 1000.0f;
        //P2
        float p2_x = control_point[2 * p][1] / 1000.0f;
        float p2_y = control_point[2 * p][2] / 1000.0f;
        //P3
        float p3_x = point[p + 1][1] / 1000.0f;
        float p3_y = point[p + 1][2] / 1000.0f;

        for (int i = 0; i < waypoint_num[p - 1]; i++){
            float t_unit = 1.0 / waypoint_num[p - 1];
            float t = (float)i * t_unit;
            float k = abs((((6 - 6 * t) * p0_x + 3 * (6 * t - 4) * p1_x - 3 * (6 * t - 2) * p2_x + 6 * t * p3_x) * ((-3 + 6 * t - 3 * t * t) * p0_y + 3 * (1 - 4 * t + 3 * t * t) * p1_y + 3 * (2 * t - 3 * t * t) * p2_y + 3 * t * t * p3_y) - ((-3 + 6 * t - 3 * t * t) * p0_x + 3 * (1 - 4 * t + 3 * t * t) * p1_x + 3 * (2 * t - 3 * t * t) * p2_x + 3 * t * t * p3_x) * ((6 - 6 * t) * p0_y + 3 * (6 * t - 4) * p1_y - 3 * (6 * t - 2) * p2_y + 6 * t * p3_y)) / pow(sqrt((pow(((-3 + 6 * t - 3 * t * t) * p0_y + 3 * (1 - 4 * t + 3 * t * t) * p1_y + 3 * (2 * t - 3 * t * t) * p2_y + 3 * t * t * p3_y), 2) + pow(((-3 + 6 * t - 3 * t * t) * p0_x + 3 * (1 - 4 * t + 3 * t * t) * p1_x + 3 * (2 * t - 3 * t * t) * p2_x + 3 * t * t * p3_x), 2))), 3));

            /*
            for (int i = 0; i < waypoint_num_sum; i++)
            {
               // writing_file << target_vel[i] << std::endl;
            }
            */
            float v;
            if (k < 0.001)
                v = max_vel;
            else
            {
                float limit_vel = sqrt(gravitational_acceleration / k) * corner_speed_rate;
                v = min(max_vel, limit_vel);
            }
            target_vel[init_index + i] = v;
        }
        init_index += waypoint_num[p - 1];
    }
    target_vel[waypoint_num_sum] = max_initial_speed;
    target_vel[0] = max_initial_speed;

    /*
    std::string filename = "test.csv";

    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);

    std::cout << "writing " << filename << "..." << std::endl;

    for (int i = 0; i <= waypoint_num_sum; i++)
    {
        writing_file << target_vel[i] << std::endl;
    }
    */

    bool *checked;
    checked = new bool[waypoint_num_sum + 1];

    for (int i = 0; i <= waypoint_num_sum; i++)
    {
        checked[i] = false;
    }

    for (int _ = 0; _ <= waypoint_num_sum; _++)
    {
        float min_v = 1000000000;
        int index = 0;
        for (int i = 0; i <= waypoint_num_sum; i++)
        {
            if (target_vel[i] < min_v && checked[i] == false)
            {
                min_v = target_vel[i];
                index = i;
            }
        }

        if (index - 1 >= 0)
        {
            float accel = (target_vel[index - 1] * target_vel[index - 1] - target_vel[index] * target_vel[index]) / (2 * path_granularity);
            if (abs(accel) > max_accel)
            {
                if (accel > 0)
                    target_vel[index - 1] = sqrt(target_vel[index] * target_vel[index] + 2 * path_granularity * max_accel);
                else
                    target_vel[index - 1] = sqrt(target_vel[index] * target_vel[index] - 2 * path_granularity * max_accel);
            }
        }

        if (index + 1 <= waypoint_num_sum)
        {
            float accel = (target_vel[index + 1] * target_vel[index + 1] - target_vel[index] * target_vel[index]) / (2 * path_granularity);
            if (abs(accel) > max_accel)
            {
                if (accel > 0)
                    target_vel[index + 1] = sqrt(target_vel[index] * target_vel[index] + 2 * path_granularity * max_accel);
                else
                    target_vel[index + 1] = sqrt(target_vel[index] * target_vel[index] - 2 * path_granularity * max_accel);
            }
        }

        checked[index] = true;
    }
    /*
    std::string filename2 = "test2.csv";

    std::ofstream writing_file2;
    writing_file2.open(filename2, std::ios::out);

    std::cout << "writing " << filename2 << "..." << std::endl;
    for (int i = 0; i <= waypoint_num_sum; i++)
    {
        writing_file2 << target_vel[i] << std::endl;
    }
    */

    delete[] checked;
}

void Path::bezier()
{
    // calc control point
    Matrix bM(2 * pnum - 2, 2 * pnum - 2);
    Matrix by_x(2 * pnum - 2, 1);
    Matrix by_y(2 * pnum - 2, 1);
    bM[1][1] = 2;
    bM[1][2] = -1;
    bM[2 * pnum - 2][2 * pnum - 3] = -1;
    bM[2 * pnum - 2][2 * pnum - 2] = 2;
    by_x[1][1] = point[1][1];
    by_y[1][1] = point[1][2];
    by_x[2 * pnum - 2][1] = point[pnum][1];
    by_y[2 * pnum - 2][1] = point[pnum][2];
    for (int j = 1; j <= pnum - 2; j++)
    {
        bM[2 * j][2 * j] = 1;
        bM[2 * j][2 * j + 1] = 1;
        bM[2 * j + 1][2 * j - 1] = 1;
        bM[2 * j + 1][2 * j] = -2;
        bM[2 * j + 1][2 * j + 1] = 2;
        bM[2 * j + 1][2 * j + 2] = -1;
        by_x[2 * j][1] = 2 * point[j + 1][1];
        by_y[2 * j][1] = 2 * point[j + 1][2];
    }
    Matrix ans_x(solve_equations(bM, by_x));
    Matrix ans_y(solve_equations(bM, by_y));
    for (int i = 1; i <= 2 * pnum - 2; i++)
    {
        control_point[i][1] = ans_x[i][1];
        control_point[i][2] = ans_y[i][1];
    }

    // calc bezier length
    for (int i = 1; i < pnum; i++)
    {
        float px[4] = {point[i][1],
                       control_point[2 * i - 1][1],
                       control_point[2 * i][1],
                       point[i + 1][1]};
        float py[4] = {point[i][2],
                       control_point[2 * i - 1][2],
                       control_point[2 * i][2],
                       point[i + 1][2]};
        point[i][4] = bezier_length(px, py);
    }
}

Matrix Path::path_func(float t)
{
    if (t < 1)
        t = 1;
    else if (t > pnum)
        t = (float)(pnum);

    int Q = (int)t;
    if (Q >= pnum)
        Q = pnum - 1;
    float tt = t - (float)Q;
    Matrix ans(4, 1);

    // calc bezier
    for (int xy = 1; xy <= 2; xy++)
    {
        ans[xy][1] = pow(1 - tt, 3) * point[Q][xy] + 3 * pow(1 - tt, 2) * tt * control_point[2 * Q - 1][xy] + 3 * (1 - tt) * pow(tt, 2) * control_point[2 * Q][xy] + pow(tt, 3) * point[Q + 1][xy];
    }

    // get theta
    ans[3][1] = point[Q][3] * (1 - tt) + point[Q + 1][3] * tt;
    // get velocity
    int index = 0;
    for (int i = 0; i <= Q-2; i++){
        index += waypoint_num[i];
    }
    index += floor((float)waypoint_num[Q-1] * tt);
    ans[4][1] = target_vel[index];
    return ans;
}

Matrix Path::solve_equations(Matrix M_, Matrix y_)
{
    // check error
    int size = M_.row_size();
    if (!(size == M_.column_size()) || !(size == y_.row_size()))
    {
        std::cerr << "err Path::solve_equations / Matrix size" << std::endl;
        exit(1);
    }

    // calc_matrix
    Matrix M(M_), y(y_);
    int *row_col = new int[size + 1]; // M[i][row_col[i]] == 1, do not use "M_index[0]"
    for (int i = 1; i <= size; i++)
    {
        float Mtemp = 0;
        for (int j = 1; j <= size; j++)
        {
            if (abs(M[i][j]) > 1e-8)
            {
                Mtemp = M[i][j];
                row_col[i] = j;
                break;
            }
        }
        if (abs(Mtemp) < 1e-8)
        { // non-invertible
            std::cerr << "err Path::solve_equations / non_invertible" << std::endl;
            exit(1);
        }
        for (int j = 1; j <= size; j++)
        {
            if (i == j)
            {
                for (int k = 1; k <= size; k++)
                {
                    M[i][k] /= Mtemp;
                }
                y[i][1] /= Mtemp;
                Mtemp = 1;
                continue;
            }
            float jtemp = M[j][row_col[i]];
            for (int k = 1; k <= size; k++)
            {
                M[j][k] -= M[i][k] * jtemp / Mtemp;
            }
            y[j][1] -= y[i][1] * jtemp / Mtemp;
        }
    }

    Matrix x(size, 1);
    for (int i = 1; i <= size; i++)
    {
        x[row_col[i]][1] = y[i][1];
    }
    delete[] row_col;
    return x;
}

float Path::length_converge(float px_[4], float py_[4], float converge)
{
    float px[4] = {0, px_[1] - px_[0], px_[2] - px_[0], px_[3] - px_[0]};
    float py[4] = {0, py_[1] - py_[0], py_[2] - py_[0], py_[3] - py_[0]};

    // calc distance
    float a2b2 = py[3] * py[3] + px[3] * px[3];
    if (a2b2 < 1e-6)
        return sqrt(a2b2);
    float distance[2] = {0, 0};
    for (int i = 0; i < 2; i++)
    {
        distance[i] = pow((py[3] * px[i + 1] - px[3] * py[i + 1]), 2) / a2b2;
    }

    // judge converge
    converge *= converge;
    if (distance[0] < converge && distance[1] < converge)
        return sqrt(a2b2);
    else
        return -1;
}

float Path::bezier_length(float px[4], float py[4], float converge)
{
    // distance pos <-> line
    float length = length_converge(px, py, converge);
    if (length > 0)
        return length;

    float newpx1[4] = {px[0],
                       (px[0] + px[1]) / 2,
                       (px[0] + 2 * px[1] + px[2]) / 4,
                       (px[0] + 3 * px[1] + 3 * px[2] + px[3]) / 8};
    float newpy1[4] = {py[0],
                       (py[0] + py[1]) / 2,
                       (py[0] + 2 * py[1] + py[2]) / 4,
                       (py[0] + 3 * py[1] + 3 * py[2] + py[3]) / 8};
    float newpx2[4] = {
        (px[0] + 3 * px[1] + 3 * px[2] + px[3]) / 8,
        (px[1] + 2 * px[2] + px[3]) / 4,
        (px[2] + px[3]) / 2,
        px[3],
    };
    float newpy2[4] = {
        (py[0] + 3 * py[1] + 3 * py[2] + py[3]) / 8,
        (py[1] + 2 * py[2] + py[3]) / 4,
        (py[2] + py[3]) / 2,
        py[3],
    };

    return bezier_length(newpx1, newpy1) + bezier_length(newpx2, newpy2);
}

float Path::costfunc(float t)
{
    Matrix p(path_func(t));
    float diff[2] = {position[0] - p[1][1], position[1] - p[2][1]};
    return diff[0] * diff[0] + diff[1] * diff[1];
}

float Path::linear_search(float (*cost)(float), float min, float max, float a, float b, float converge)
{
    if (a < min)
        a = min;
    if (b > max)
        b = max;

    // set a-b
    float delta = b - a;
    float acb[3][2] = {{a, cost(a)},
                       {(a + b) / 2, cost((a + b) / 2)},
                       {b, cost(b)}}; // [t][f(t)]
    while (1)
    {
        if (acb[0][1] < acb[1][1] && acb[1][1] < acb[2][1])
        {
            acb[2][0] = acb[1][0];
            acb[2][1] = acb[1][1];
            acb[1][0] = acb[0][0];
            acb[1][1] = acb[0][1];
            acb[0][0] = acb[2][0] - delta;
            acb[0][1] = cost(acb[0][0]);
            if (acb[0][0] < min)
            {
                acb[0][0] = min;
                acb[0][1] = cost(acb[0][0]);
                acb[2][0] = acb[0][0] + delta;
                break;
            }
        }
        else if (acb[0][1] > acb[1][1] && acb[1][1] > acb[2][1])
        {
            acb[0][0] = acb[1][0];
            acb[0][1] = acb[1][1];
            acb[1][0] = acb[2][0];
            acb[1][1] = acb[2][1];
            acb[2][0] = acb[0][0] + delta;
            acb[2][1] = cost(acb[2][0]);
            if (acb[2][0] > max)
            {
                acb[2][0] = max;
                acb[2][1] = cost(acb[2][0]);
                acb[0][0] = acb[2][0] - delta;
                break;
            }
        }
        else
        {
            break;
        }
    }

    // golden section search
    float t_ft[4][2]; // [ta,t1,t2,tb][t,f(t)]
    float gamma = (3.0f - sqrt(5.0f)) / 2.0f;
    t_ft[0][0] = acb[0][0];
    t_ft[3][0] = acb[2][0];
    t_ft[1][0] = t_ft[0][0] + gamma * (t_ft[3][0] - t_ft[0][0]);
    t_ft[2][0] = t_ft[0][0] + (1 - gamma) * (t_ft[3][0] - t_ft[0][0]);
    t_ft[1][1] = cost(t_ft[1][0]);
    t_ft[2][1] = cost(t_ft[2][0]);
    while (abs(t_ft[0][0] - t_ft[3][0]) > converge)
    {
        if (t_ft[1][1] < t_ft[2][1])
        {
            t_ft[3][0] = t_ft[2][0];
            t_ft[2][0] = t_ft[1][0];
            t_ft[1][0] = t_ft[0][0] + gamma * (t_ft[3][0] - t_ft[0][0]);
            t_ft[2][1] = t_ft[1][1];
            t_ft[1][1] = cost(t_ft[1][0]);
        }
        else
        {
            t_ft[0][0] = t_ft[1][0];
            t_ft[1][0] = t_ft[2][0];
            t_ft[2][0] = t_ft[0][0] + (1 - gamma) * (t_ft[3][0] - t_ft[0][0]);
            t_ft[1][1] = t_ft[2][1];
            t_ft[2][1] = cost(t_ft[2][0]);
        }
    }
    return (t_ft[1][0] + t_ft[2][0]) / 2.0f;
}

float Path::linear_search(float min, float max, float a, float b, float converge)
{
    if (a < min)
        a = min;
    if (b > max)
        b = max;

    // set a-b
    float delta = b - a;
    float acb[3][2] = {{a, costfunc(a)},
                       {(a + b) / 2, costfunc((a + b) / 2)},
                       {b, costfunc(b)}}; // [t][f(t)]
    while (1)
    {
        if (acb[0][1] < acb[1][1] && acb[1][1] < acb[2][1])
        {
            acb[2][0] = acb[1][0];
            acb[2][1] = acb[1][1];
            acb[1][0] = acb[0][0];
            acb[1][1] = acb[0][1];
            acb[0][0] = acb[2][0] - delta;
            acb[0][1] = costfunc(acb[0][0]);
            if (acb[0][0] < min)
            {
                acb[0][0] = min;
                acb[0][1] = costfunc(acb[0][0]);
                acb[2][0] = acb[0][0] + delta;
                break;
            }
        }
        else if (acb[0][1] > acb[1][1] && acb[1][1] > acb[2][1])
        {
            acb[0][0] = acb[1][0];
            acb[0][1] = acb[1][1];
            acb[1][0] = acb[2][0];
            acb[1][1] = acb[2][1];
            acb[2][0] = acb[0][0] + delta;
            acb[2][1] = costfunc(acb[2][0]);
            if (acb[2][0] > max)
            {
                acb[2][0] = max;
                acb[2][1] = costfunc(acb[2][0]);
                acb[0][0] = acb[2][0] - delta;
                break;
            }
        }
        else
        {
            break;
        }
    }

    // golden section search
    float t_ft[4][2]; // [ta,t1,t2,tb][t,f(t)]
    float gamma = (3.0f - sqrt(5.0f)) / 2.0f;
    t_ft[0][0] = acb[0][0];
    t_ft[3][0] = acb[2][0];
    t_ft[1][0] = t_ft[0][0] + gamma * (t_ft[3][0] - t_ft[0][0]);
    t_ft[2][0] = t_ft[0][0] + (1 - gamma) * (t_ft[3][0] - t_ft[0][0]);
    t_ft[1][1] = costfunc(t_ft[1][0]);
    t_ft[2][1] = costfunc(t_ft[2][0]);
    while (abs(t_ft[0][0] - t_ft[3][0]) > converge)
    {
        if (t_ft[1][1] < t_ft[2][1])
        {
            t_ft[3][0] = t_ft[2][0];
            t_ft[2][0] = t_ft[1][0];
            t_ft[1][0] = t_ft[0][0] + gamma * (t_ft[3][0] - t_ft[0][0]);
            t_ft[2][1] = t_ft[1][1];
            t_ft[1][1] = costfunc(t_ft[1][0]);
        }
        else
        {
            t_ft[0][0] = t_ft[1][0];
            t_ft[1][0] = t_ft[2][0];
            t_ft[2][0] = t_ft[0][0] + (1 - gamma) * (t_ft[3][0] - t_ft[0][0]);
            t_ft[1][1] = t_ft[2][1];
            t_ft[2][1] = costfunc(t_ft[2][0]);
        }
    }
    return (t_ft[1][0] + t_ft[2][0]) / 2.0f;
}

void Path::listen_goal_position(float &x, float &y, const bool &forward)
{
    if(forward){
        x = point[pnum][1];
        y = point[pnum][2];
    }
    else
    {
        x = point[1][1];
        y = point[1][2];
    }
}

Matrix Path::pure_pursuit(float posx, float posy, float body_theta, bool foward, float reset_t)
{
    using namespace std;
    position[0] = posx;
    position[1] = posy;

    if (reset_t > 0)
        ref_t = reset_t;
    ref_t = linear_search(1, (float)pnum, ref_t - 0.2f, ref_t + 0.2f);
    float aim_t;
    if (foward)
    {
        aim_t = ref_t + PURSUIT_AIM;
    }
    else
    {
        aim_t = ref_t - PURSUIT_AIM;
    }
    if (aim_t > pnum)
    {
        aim_t = (float)pnum;
    }
    else if (aim_t < 1)
    {
        aim_t = 1;
    }

    Matrix ref(path_func(ref_t));
    Matrix aim(path_func(aim_t));
    Matrix direction(2, 1);
    direction[1][1] = aim[1][1] - posx;
    direction[2][1] = aim[2][1] - posy;
    //float direction[2] = {aim[1][1] - posx, aim[2][1] - posy};
    //float norm = sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
    float norm = sqrt(direction[1][1] * direction[1][1] + direction[2][1] * direction[2][1]);
    if (norm > 1e-5)
    {
        direction[1][1] *= ref[4][1] / norm;
        direction[2][1] *= ref[4][1] / norm;
    }
    else
    {
        direction[1][1] = 0;
        direction[2][1] = 0;
    }

    Matrix Rotation_Matrix(2, 2);
    Rotation_Matrix[1][1] = cos(body_theta);
    Rotation_Matrix[1][2] = sin(body_theta);
    Rotation_Matrix[2][1] = -1*sin(body_theta);
    Rotation_Matrix[2][2] = cos(body_theta);

    // calc vx, vy
    direction = Rotation_Matrix * direction;
    

    // calc omega
    float omega = ref[3][1] - body_theta;

    Matrix ans(5, 1);
    ans[1][1] = direction[1][1];
    ans[2][1] = direction[2][1];
    ans[3][1] = omega;
    ans[4][1] = ref_t;
    ans[5][1] = ref[3][1];
    return ans;
}