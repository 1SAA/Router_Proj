#include <map>
#include <queue>
#include <algorithm>
#include <cmath>

#include "solver.h"
#include "flute.h"
#include "utils.h"

/**
 * @brief Init 2d routing infomation. i.e. generate STtree, twopins
 * @param netid 
 * @return 2d routing info
 */
RouteInfo Solver::init2d(int netid) {
    Net &net = Nets[netid];
    printNet(net);
    int deg = net.pins.size();
    int *xs = new DTYPE [deg];
    int *ys = new DTYPE [deg];
    std::map<std::pair<int, int>, std::vector<int> > coord2pin;

    for (int i = 0, n = net.pins.size(); i < n; ++i) {
        CellInst &inst = cInsts[net.pins[i].inst];
        xs[i] = inst.position.x;
        ys[i] = inst.position.y;
        coord2pin[std::make_pair(xs[i], ys[i])].push_back(i);
    }

    const int FLUTE_ACC = 3;
    Tree t = flute(deg, xs, ys, FLUTE_ACC);
    printtree(t);
    int treeSize = t.deg * 2 - 2;

    RouteInfo info;
    info.netid = netid;
    std::map<std::pair<int, int>, int> coord2id;

    int size = 0;

    for (int i = 0; i < treeSize; ++i) {
        std::pair<int, int> pos[2];
        pos[0] = std::make_pair(t.branch[i].x, t.branch[i].y);
        pos[1] = std::make_pair(t.branch[t.branch[i].n].x, t.branch[t.branch[i].n].y);

        for (int j = 0; j < 2; ++j) {
            if (coord2id.find(pos[j]) == coord2id.end()) {
                coord2id[pos[j]] = size;

                info.tree.nodes.push_back(Node());
                auto node = info.tree.nodes.back();

                node.id = size;        
                node.layers.resize(coord2pin[pos[j]].size(), 0);
                node.hasPin = node.layers.size() > 0;
                node.x = pos[j].first, node.y = pos[j].second;

                size++;        
            }
        }

        if (pos[0] != pos[1]) {
            auto child_id = coord2id[pos[0]];
            auto parent_id = coord2id[pos[1]];
            info.tree.nodes[parent_id].neighbor.push_back(Edge(
                info.tree.nodes[parent_id],
                info.tree.nodes[child_id],
                std::vector<Segment>(),
                std::vector<DPInfo>()
            ));
        } else {
            info.tree.root = coord2id[pos[0]];
        }
    }

    delete xs;
    delete ys;

    return info;
}

/**
 * @brief get estimated 2D congestion, i.e. min(supply - demand) along the route
 *        H congestion and V congestion are seperated
 * @return int congestion
 */
int Solver::get2DCon(Point st, Point en) {
    return DataBase.query2DCon(Segment(st, en));
}

bool Solver::mazeRouting(Edge &twopin) {
    int dir[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; // 0, 2 Horizontal 1, 4 Vertical
    std::vector<std::vector<int> > Dist(numRow + 1, std::vector<int>(numCol + 1, -1));
    std::deque<std::pair<int, int> > Q;

    auto &st = twopin.child, &en = twopin.parent;
    auto calc_dist = [&] (int x, int y) {
        return std::abs(x - en.x) + std::abs(y - en.y);
    };

    Q.push_back(std::make_pair(st.x, st.y));
    Dist[st.x][st.y] = 0;

    while (!Q.empty()) {
        auto now = Q.front();
        Q.pop_front();
        if (now.first == en.x && now.second == en.y)
            break;
        for (int d = 0; d < 4; ++d) {
            int nx = now.first + dir[d][0], ny = now.second + dir[d][0];
            if (nx <= 0 || nx > numRow || ny <= 0 || ny > numCol)
                continue;
            if (Dist[nx][ny] != -1)
                continue;
            if (DataBase.get2DCon(Point(nx, ny), d & 1) <= 0 ||
                DataBase.get2DCon(Point(now.first, now.second), d & 1) <= 0)
                continue;
            Dist[nx][ny] = Dist[now.first][now.first] + 1;
            if (calc_dist(nx, ny) < calc_dist(st.x, st.y))
                Q.push_front(std::make_pair(nx, ny));
            else
                Q.push_back(std::make_pair(nx, ny)); // detour
        }
    }
    
    if (Dist[en.x][en.y] == -1)
        return false;

    int nowx = en.x, nowy = en.y;
    int lastx = en.x, lasty = en.y;
    int lastHV = -1;

    while (nowx != st.x && nowy != st.y) {
        for (int d = 0; d < 4; ++d) {
            int nx = nowx + dir[d][0], ny = nowy + dir[d][1];
            if (nx <= 0 || nx > numRow || ny <= 0 || ny > numCol)
                continue;
            if (DataBase.get2DCon(Point(nx, ny), d & 1) <= 0 ||
                DataBase.get2DCon(Point(nowx, nowy), d & 1) <= 0)
                continue;
            if (Dist[nx][ny] + 1 != Dist[nowx][nowy])
                continue;

            if (lastHV != (d & 1)) {
                twopin.segs.push_back(Segment(Point(nowx, nowy), Point(lastx, lasty)));
                lastx = nowx, lasty = nowy;
            }
            
            nowx = nx, nowy = ny;
            lastHV = d & 1;
            break;
        }
    }
    twopin.segs.push_back(Segment(Point(st.x, st.y), Point(lastx, lasty)));
    std::reverse(twopin.segs.begin(), twopin.segs.end());
    return true;
}

bool Solver::route2pin2d(Edge &twopin) {
    twopin.segs.clear();
    /// calculate bounding box
    Node &st = twopin.child, &en = twopin.parent;
    int lX = std::min(en.x, st.x);
    int lY = std::min(en.y, st.y);
    int hX = std::max(en.x, st.x);
    int hY = std::max(en.y, st.y);

    const double ratio = 1.5; /// increase the bounding box by a ratio
    int midX = (lX + hX) / 2, wX = hX - lX;
    int midY = (lY + hY) / 2, wY = hY - lY;

    lX = std::max((int)floor(midX - wX * ratio / 2), 1), hX = std::min((int)ceil(midX + wX * ratio / 2), numRow);
    lY = std::max((int)floor(midY - wY * ratio / 2), 1), hY = std::min((int)ceil(midY + wY * ratio / 2), numCol);
    dbg_print("%d %d %d %d\n", lX, hX, lY, hY);

    int min_cost = INF, max_con = INF;
    std::vector<Segment> best_route;
    for (int bx = lX; bx <= hX; ++bx)
        for (int by = lY; by <= hY; ++by) {
            /// two possible routes from st -> (bx, by) with identical cost
            /// two possible routes from (bx, by) -> en with identical cost
            /// choose the one with minimum congestion

            int cost = std::abs(bx - st.x) + std::abs(bx - en.x) + 
                       std::abs(by - st.y) + std::abs(by - en.y); /// TODO retouch this cost function
            
            /// check 2D congestion
            int route1_con = std::min(get2DCon(Point(st.x, st.y), Point(st.x, by)), get2DCon(Point(st.x, by), Point(bx, by)));
            int route2_con = std::min(get2DCon(Point(st.x, st.y), Point(bx, st.y)), get2DCon(Point(bx, st.y), Point(bx, by)));
            int route3_con = std::min(get2DCon(Point(bx, by), Point(en.x, by)), get2DCon(Point(en.x, by), Point(en.x, en.y)));
            int route4_con = std::min(get2DCon(Point(bx, by), Point(bx, en.y)), get2DCon(Point(bx, en.y), Point(en.x, en.y)));

            int total_con = std::min(std::max(route1_con, route2_con), std::max(route3_con, route4_con));
            if (total_con <= 0)
                continue;
            if (min_cost < cost || (min_cost == cost && max_con >= total_con))
                continue;

            std::vector<Segment> cur;
            if (route1_con > route2_con) {
                cur.push_back(Segment(Point(st.x, st.y), Point(st.x, by)));
                cur.push_back(Segment(Point(st.x, by), Point(bx, by)));
            } else {
                cur.push_back(Segment(Point(st.x, st.y), Point(bx, st.y)));
                cur.push_back(Segment(Point(bx, st.y), Point(bx, by)));
            }
            if (route3_con > route4_con) {
                cur.push_back(Segment(Point(bx, by), Point(en.x, by)));
                cur.push_back(Segment(Point(en.x, by), Point(en.x, en.y)));
            } else {
                cur.push_back(Segment(Point(bx, by), Point(bx, en.y)));
                cur.push_back(Segment(Point(bx, en.y), Point(en.x, en.y)));
            }

            min_cost = cost;
            max_con = total_con;
            best_route = cur;
        }

    if (min_cost == INF) {
        if (!mazeRouting(twopin) )
            return false;
        return true;
    }
    twopin.segs = best_route;
    return true;
}

bool Solver::route2d(RouteInfo &info) {
    for (auto twopin : info.edges)
        if (route2pin2d(*twopin) == false) {
            dbg_print("(%d, %d) <--> (%d, %d)", twopin->child.x, twopin->child.y, twopin->parent.x, twopin->parent.y);
            return false;
        }
    return true;
}

int Solver::get3DCon(Point p) {
    return DataBase.get3DCon(p);
}

/**
 * @brief get the real 3D congestion on path (st, en)
 * @return minimum of {supply - demand} on the path
 */
int Solver::get3DCon(Point st, Point en) {
    return DataBase.query3DCon(Segment(st, en));
}

/**
 * @brief dynamic programming on an two-pin edge.
 *        child's dp array has been computed.
 *        each whole seg is in one layer
 * @param edge
 * @param min_layer
 * @return void
 */
void Solver::edge_dp(Edge &edge, int min_layer) {
    edge.dp_segs.resize(edge.segs.size());
    int i = 0;
    for (auto &seg : edge.segs) {
        Point &st = seg.spoint, &en = seg.epoint;
        edge.dp_segs[i].cost.resize(numLayer + 1);
        edge.dp_segs[i].prev.resize(numLayer + 1);

        for (int h = min_layer; h <= numLayer; ++h) {
            edge.dp_segs[i].cost[h] = 1e9;
            for (int k = min_layer; k <= numLayer; ++k) {
                /// (st.x, st.y, k) -> (st.x, st.y, h) -> (en.x, en.y, h)

                int dir = st.x == en.x ? 0 : 1;
                if ((k & 1) != dir) 
                /// layer with odd ID always be horizontal
                    continue;
                
                /// check congestion
                int con = get3DCon(Point(st.x, st.y, k), Point(st.x, st.y, h));
                con = std::min(con, get3DCon(Point(st.x, st.y, h), Point(en.x, en.y, h)));
                if (con <= 0)
                    continue;
                
                float prev_value = 0.0;
                if (i == 0)
                    prev_value = edge.child.dp.cost[k];
                else
                    prev_value = edge.dp_segs[i - 1].cost[k];

                float Zdist = h < k ? 
                              powerPrefixSum[k - 1] - powerPrefixSum[h - 1] : 
                              powerPrefixSum[h] - powerPrefixSum[k];

                float HVdist = (Point(st.x, st.y, h) - Point(en.x, en.y, h)).norm1() * Layers[h].powerFactor;

                float total_value = Zdist + HVdist + prev_value;

                if (total_value < edge.dp_segs[i].cost[h]) {
                    edge.dp_segs[i].cost[h] = total_value;
                    edge.dp_segs[i].prev[h] = k;
                }
            }
        }
        i++;
    }
}

/**
 * @brief Using dynamic programming to do layer assignment
 *        Assume two-pin paths in a net are independent
 *        Complexity O(tree.size * L^2)
 * @param node root of the current sub-tree
 * @param min_layer min layer constraint
 * @return void
 */
void Solver::assign_layer_dp(Node &node, int min_layer) {
    node.dp.cost.resize(numLayer + 1, F_INF);

    std::vector<std::vector<float> > mindp(numLayer + 1, std::vector<float>(numLayer + 1, 0.0));

    for (auto &edge : node.neighbor) {
        assign_layer_dp(edge.child, min_layer);
        edge_dp(edge, min_layer);
        std::vector<std::vector<float> > intv(numLayer + 1, std::vector<float>(numLayer + 1, 0.0));
        for (int i = 1; i <= numLayer; ++i)
            intv[i][i] = edge.dp_segs.front().cost[i];
        for (int i = 1; i <= numLayer; ++i)
            for (int j = i + 1; j <= numLayer; ++j) {
                intv[i][j] = std::min(intv[i][j - 1], edge.dp_segs.front().cost[j]);
                mindp[i][j] += intv[i][j];
            }
    }

    int lmax = numLayer, rmin = 1;

    if (node.hasPin) {
        for (auto pin : node.layers) {
            lmax = std::min(pin, lmax);
            rmin = std::max(pin, rmin);
        }
    }

    node.intv_cost.resize(numLayer + 1, std::vector<float>(numLayer + 1, F_INF));

    for (int l = 1; l <= lmax; ++l)
        for (int r = std::max(l, rmin); r <= numLayer; ++r) {
            if (get3DCon(Point(node.x, node.y, l), Point(node.x, node.y, r)) <= 0)
                continue;
            float via_cost = powerPrefixSum[r] - powerPrefixSum[l - 1];
            via_cost += mindp[l][r];
            node.intv_cost[l][r] = via_cost;
        }

    for (int l = 1; l <= lmax; ++l) {
        float minvalue = F_INF;

        for (int r = numLayer; r >= std::max(l, rmin); --r) {

            minvalue = std::min(minvalue, node.intv_cost[l][r]);

            if (r >= min_layer)
                node.dp.cost[r] = std::min(node.dp.cost[r], minvalue);
        }
    }
}

int Solver::assign_layer_for_edge(Edge &edge, int startLayer, std::vector<Segment> &segs_3d) {
    int curLayer = startLayer;
    for (unsigned i = edge.segs.size() - 1; i >= 0; --i) {
        auto &seg = edge.segs[i];
        auto st = seg.spoint;
        auto en = seg.epoint;

        auto &dp = edge.dp_segs[i];
        int nextLayer = dp.prev[curLayer];

        /// (st.x, st.y, next) -> (st.x, st.y, cur) -> (en.x, en.y, cur)
        segs_3d.push_back(Segment({st.x, st.y, nextLayer}, {st.x, st.y, curLayer}));
        segs_3d.push_back(Segment({st.x, st.y, curLayer}, {en.x, en.y, curLayer}));
        curLayer = nextLayer;
    }
    return curLayer;
}

/**
 * @brief Retrieve the routing segment from the previous dynamic programming
 * @param node root of the current sub-tree
 * @param min_layer min layer constraint
 * @return void
 */
void Solver::assign_layer_compute_seg(Node &node, int curLayer, std::vector<Segment> &segs_3d) {
    int bestL = -1, bestR = -1;
    float mincost = F_INF;

    for (int l = 1; l <= curLayer; ++l)
        for (int r = curLayer; r <= numLayer; ++r)
            if (node.intv_cost[l][r] < mincost) {
                mincost = node.intv_cost[l][r];
                bestL = l, bestR = r;
            }
    
    assert(bestL != -1 && bestR != -1);

    segs_3d.push_back(Segment(Point(node.x, node.y, bestL), 
                              Point(node.x, node.y, bestR)));

    for (auto &edge :node.neighbor) {
        int bestLayer = -1;
        float cost = F_INF;

        for (int i = bestL; i <= bestR; ++i)
            if (edge.dp_segs.front().cost[i] < cost) {
                bestLayer = i;
                cost = edge.dp_segs.front().cost[i];
            }

        assert(bestLayer != -1);

        int nextLayer = assign_layer_for_edge(edge, bestLayer, segs_3d);
        assign_layer_compute_seg(edge.child, nextLayer, segs_3d);
    }
}

bool Solver::route3d(RouteInfo &info) {
    assign_layer_dp(info.tree.nodes[info.tree.root], Nets[info.netid].minConstraint);
    Node &root_node = info.tree.nodes[info.tree.root];

    float bestValue = F_INF;
    int bestLayer = 0;    
    for (int l = 1; l < numLayer; ++l)
        if (root_node.dp.cost[l] < bestValue) {
            bestValue = root_node.dp.cost[l];
            bestLayer = l;
        }
    if (bestLayer == 0)
        return false;
    assign_layer_compute_seg(root_node, bestLayer, info.segs_3d);
    return true;
}