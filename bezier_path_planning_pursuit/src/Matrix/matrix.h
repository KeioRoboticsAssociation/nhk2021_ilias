// matrix.h

#ifndef MATRIX_H
#define MATRIX_H

#include <ros/ros.h>

class Matrix
{

    int row;
    int column;

    float **p_top;

public:
    Matrix(int i = 1, int j = 1);
    Matrix(const Matrix &cp);

    ~Matrix();

    int row_size() { return (row); }
    int column_size() { return (column); }

    void change_size(int i, int j);

    float *&operator[](int i) { return (p_top[i]); }
    Matrix operator=(const Matrix &a);
    Matrix operator+(const Matrix &a);
    Matrix operator-(const Matrix &a);
    Matrix operator*(const Matrix &a);

    friend Matrix operator*(const Matrix &a, float b);
    friend Matrix operator*(float b, const Matrix &a);

    void unit_matrix();
    Matrix transposed();

    void show();
};

#endif