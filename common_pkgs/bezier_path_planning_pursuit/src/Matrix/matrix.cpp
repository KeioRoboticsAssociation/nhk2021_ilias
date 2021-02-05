// matrix.cpp

#include <iostream>
#include <iomanip>

#include "matrix.h"
using namespace std;

Matrix::Matrix(int i, int j)
{
    if (i < 1 || j < 1)
    {
        cerr << "err Matrix::Matrix" << endl;
        exit(1);
    }

    row = i;
    column = j;

    p_top = new float *[row + 1];
    *p_top = new float[row * column + 1];

    for (int k = 1; k <= row; k++)
        *(p_top + k) = *p_top + ((k - 1) * column);

    for (int k1 = 1; k1 <= row; k1++)
    {
        for (int k2 = 1; k2 <= column; k2++)
        {
            p_top[k1][k2] = 0;
        }
    }
}

Matrix::Matrix(const Matrix &cp)
{
    row = cp.row;
    column = cp.column;

    p_top = new float *[row + 1];
    *p_top = new float[row * column + 1];

    for (int k = 1; k <= row; k++)
        *(p_top + k) = *p_top + ((k - 1) * column);

    for (int k1 = 1; k1 <= row; k1++)
    {
        for (int k2 = 1; k2 <= column; k2++)
        {
            p_top[k1][k2] = cp.p_top[k1][k2];
        }
    }
}

Matrix::~Matrix()
{
    delete[] * p_top;
    delete[] p_top;
}

void Matrix::change_size(int i, int j)
{
    if (i < 1 || j < 1)
    {
        cerr << "err Matrix::change_size" << endl;
        exit(1);
    }

    delete[] * p_top;
    delete[] p_top;

    row = i;
    column = j;

    p_top = new float *[row + 1];
    *p_top = new float[row * column + 1];

    for (int k = 1; k <= row; k++)
        *(p_top + k) = *p_top + ((k - 1) * column);

    for (int k1 = 1; k1 <= row; k1++)
    {
        for (int k2 = 1; k2 <= column; k2++)
        {
            p_top[k1][k2] = 0;
        }
    }
}

Matrix Matrix::operator=(const Matrix &a)
{
    if (row != a.row || column != a.column)
    {
        change_size(a.row, a.column);
    }

    for (int i = 1; i <= row; i++)
    {
        for (int j = 1; j <= column; j++)
        {
            p_top[i][j] = a.p_top[i][j];
        }
    }
    return (*this);
}

Matrix Matrix::operator+(const Matrix &a)
{
    if (row != a.row || column != a.column)
    {
        cerr << "err Matrix::operator+" << endl;
        cerr << "  not equal matrix size" << endl;
        exit(0);
    }

    Matrix r(row, column);
    for (int i = 1; i <= row; i++)
    {
        for (int j = 1; j <= column; j++)
        {
            r.p_top[i][j] = p_top[i][j] + a.p_top[i][j];
        }
    }
    return (r);
}

Matrix Matrix::operator-(const Matrix &a)
{
    if (row != a.row || column != a.column)
    {
        cerr << "err Matrix::operator-" << endl;
        cerr << "  not equal matrix size" << endl;
        exit(0);
    }

    Matrix r(row, column);
    for (int i = 1; i <= row; i++)
    {
        for (int j = 1; j <= column; j++)
        {
            r.p_top[i][j] = p_top[i][j] - a.p_top[i][j];
        }
    }
    return (r);
}

Matrix Matrix::operator*(const Matrix &a)
{
    if (column != a.row)
    {
        cerr << "err Matrix::operator*" << endl;
        cerr << "  not equal matrix size" << endl;
        exit(0);
    }

    Matrix r(row, a.column);
    for (int i = 1; i <= row; i++)
    {
        for (int j = 1; j <= a.column; j++)
        {
            for (int k = 1; k <= column; k++)
            {
                r.p_top[i][j] += p_top[i][k] * a.p_top[k][j];
            }
        }
    }
    return (r);
}

Matrix operator*(const Matrix &a, float b)
{
    Matrix r(a.row, a.column);
    for (int i = 1; i <= a.row; i++)
    {
        for (int j = 1; j <= a.column; j++)
        {
            r[i][j] = b * a.p_top[i][j];
        }
    }
    return (r);
}
Matrix operator*(float b, const Matrix &a)
{
    Matrix r(a.row, a.column);
    for (int i = 1; i <= a.row; i++)
    {
        for (int j = 1; j <= a.column; j++)
        {
            r[i][j] = b * a.p_top[i][j];
        }
    }
    return (r);
}

void Matrix::unit_matrix()
{
    if (row != column)
    {
        cerr << "err in Matrix::unit_matrix()" << endl;
        exit(0);
    }

    int n = row;
    float **a = p_top;
    for (int i = 1; i <= n; i++)
    {
        for (int j = 1; j <= n; j++)
        {
            a[i][j] = 0;
            if (i == j)
                a[i][j] = 1;
        }
    }
}

Matrix Matrix::transposed()
{
    Matrix t(column, row);
    float **a = p_top;

    for (int i = 1; i <= row; i++)
    {
        for (int j = 1; j <= column; j++)
        {
            t[j][i] = a[i][j];
        }
    }
    return (t);
}

void Matrix::show()
{
    for (int i = 1; i <= row_size(); i++)
    {
        std::cout << "| ";
        for (int j = 1; j <= column_size(); j++)
        {
            std::cout << setw(12) << p_top[i][j] << " ";
        }
        std::cout << "|\n";
    }
    std::cout << "\n";
}
