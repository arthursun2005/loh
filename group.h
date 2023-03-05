#pragma once

#include <bits/extc++.h>
using namespace std;

#include <Eigen/Dense>
using Vec2 = Eigen::Vector2f;

#include <boost/sort/sort.hpp>

using ll = long long;

constexpr int nthreads = 20;

auto now()
{
    return chrono::system_clock::now();
}

float inv_sqrt(float x)
{
	static_assert(numeric_limits<float>::is_iec559);
	float y = bit_cast<float>(0x5f3759df - (bit_cast<uint32_t>(x) >> 1));
	return y * (1.5f - (x * 0.5f * y * y));
}

using CollisionInfo = tuple<size_t, size_t, float, Vec2>;

struct FluidGroup
{
    vector<Vec2> pos, vel, acc, newacc;
    vector<float> u;
    vector<size_t> idx;
    vector<int> level;
    float radius, diat;

    float rep, visc;
    float *u_data;

    vector<CollisionInfo> collisions;

    FluidGroup() : radius(3.0f), rep(3000.0f), visc(3.0f) {}

    void add(float x, float y)
    {
        pos.push_back({x, y});
        vel.push_back({0.0f, 0.0f});
        acc.push_back({0.0f, 0.0f});
        newacc.push_back({0.0f, 0.0f});
        u.push_back({0.0f});
        idx.push_back(idx.size());
        level.push_back(0);
    }

    void detect(auto i, auto j, vector<CollisionInfo> &b)
    {
        if (abs(pos[j].y() - pos[i].y()) > diat) return;
        Vec2 d = pos[j] - pos[i];
        float g = d.squaredNorm();
        if (g < diat * diat)
        {
            float h = inv_sqrt(g);
            float w = 1.0f - 1.0f / (h * diat);
            b.push_back({i, j, w, d * h});
        }
    }

    void attract(float x, float y, float s)
    {
        Vec2 p(x, y);
        for (size_t i = 0; i < pos.size(); i++)
        {
            Vec2 d = p - pos[i];
            float g = d.norm() + numeric_limits<float>::epsilon();
            vel[i] += d * s;
            Vec2 n = d.normalized();
            // vel[i] += d * s + Vec2(n.y(), -n.x()) / g * s * 2000.0f;
            // vel[i] += (d + Vec2(n.y(), -n.x()) * 20.0f) * s;
        }
    }

    void step(float dt)
    {
        size_t n = idx.size();
        if (not n)
            return;
        for (size_t i = 0; i < n; i++)
        {
            pos[i] += vel[i] * dt + acc[i] * dt * dt * 0.5;
        }

        diat = radius * 2.0f;
        for (size_t i = 0; i < n; i++)
        {
            u[i] = 0.0f;
            level[i] = floor(pos[i].y() / diat);
        }
        boost::sort::spinsort(idx.begin(), idx.end(), [&] (auto i, auto j) {
            if (level[i] != level[j])
                return level[i] < level[j];
            return pos[i].x() < pos[j].x();
        });
        collisions.clear();
        vector<CollisionInfo> b[nthreads];
        u_data = u.data();
        vector<thread> threads(nthreads);
        size_t block_size = n / nthreads;
        for (int tid = 0; tid < nthreads; tid++)
        {
            size_t begin = tid * block_size;
            size_t end = tid == nthreads - 1 ? n : (begin + block_size);
            threads[tid] = thread([&] (size_t begin, size_t end, int tid) {
                size_t ptr = begin;
                for (size_t ii = begin; ii < end; ii++)
                {
                    auto i = idx[ii];
                    for (size_t jj = ii + 1; jj < n; jj++)
                    {
                        auto j = idx[jj];
                        if (level[i] != level[j]) break;
                        if (pos[j].x() - pos[i].x() > diat) break;
                        detect(i, j, b[tid]);
                    }

                    while (ptr < n and level[idx[ptr]] <= level[i]) ptr++;
                    while (ptr < n and level[idx[ptr]] == level[i] + 1 and pos[i].x() - pos[idx[ptr]].x() > diat) ptr++;

                    for (size_t jj = ptr; jj < n; jj++)
                    {
                        auto j = idx[jj];
                        if (level[i] + 1 != level[j]) break;
                        if (pos[j].x() - pos[i].x() > diat) break;
                        detect(i, j, b[tid]);
                    }
                }
            }, begin, end, tid);
        }
        for (int i = 0; i < nthreads; i++)
        {
            threads[i].join();
        }
        for (int i = 0; i < nthreads; i++)
        {
            collisions.insert(collisions.end(), b[i].begin(), b[i].end());
        }
        for (auto &[i, j, w, n] : collisions)
        {
            u[i] += w;
            u[j] += w;
        }
        for (size_t i = 0; i < n; i++)
        {
            u[i] = max(u[i] - 2.0f, 0.0f);
            newacc[i] = {0.0, 0.0};
        }
        for (auto &[i, j, w, n] : collisions)
        {
            float f = (u[i] + u[j]) * w * radius * rep;
            // Vec2 d = n * f;//(n * f - visc * (vel[j] - vel[i]));
            Vec2 d = (n * f - visc * (vel[j] - vel[i]));
            newacc[i] -= d;
            newacc[j] += d;

            // d = -dt * visc * (vel[j] - vel[i]);
            // vel[i] -= d;
            // vel[j] += d;
        }
        for (size_t i = 0; i < n; i++)
        {
            // pos[i] += vel[i] * dt;
            vel[i] += (acc[i] + newacc[i]) * dt * 0.5;
            acc[i] = newacc[i];
        }
    }
};
