#include <map>
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
    int deg = net.pins.size();
    int *xs = new DTYPE [deg];
    int *ys = new DTYPE [deg];
    for (int i = 0, n = net.pins.size(); i < n; ++i) {
        CellInst &inst = cInsts[net.pins[i].inst];
        xs[i] = inst.position.x;
        ys[i] = inst.position.y;
    }

    const int FLUTE_ACC = 3;
    Tree t = flute(deg, xs, ys, FLUTE_ACC);
    
    int treeSize = t.deg * 2 - 2;

    RouteInfo info;
    info.netid = netid;
    info.tree.nodes.resize(treeSize);
    std::map<std::pair<int, int>, std::vector<int> > coord2id;

    for (int i = 0; i < treeSize; ++i) {
        Node &node = info.tree.nodes[i];
        node.x = t.branch[i].x;
        node.y = t.branch[i].y;
        if (t.branch[i].n != i) {
            info.tree.nodes[t.branch[i].n].neighbor.push_back({
                info.tree.nodes[t.branch[i].n],
                node,
                std::vector<Segment>(),
                std::vector<DPInfo>()
            });
            info.edges.push_back(&info.tree.nodes[t.branch[i].n].neighbor.back());
            //info.twopins.push_back((TwoPin2D){node, info.tree.nodes[t.branch[i].n], std::vector<Segment>()});
        } else {
            info.tree.root = i;
        }
        coord2id[std::make_pair(node.x, node.y)].push_back(i);
    }

    for (int i = 0, n = net.pins.size(); i < n; ++i) {
        CellInst &inst = cInsts[net.pins[i].inst];
        int x = inst.position.x, y = inst.position.y;
        auto iter = coord2id.find(std::make_pair(x, y));
        DEBUG(iter != coord2id.end());
        int id = coord2id[std::make_pair(x, y)].back();

        info.tree.nodes[id].isPin = 1;
        info.tree.nodes[id].layer = net.pins[i].layer;
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
    return false;
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
    lX = midX - wX * ratio / 2, hX = midX + wX * ratio / 2;
    lY = midY - wY * ratio / 2, hY = midY + wY * ratio / 2;
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
        /// TODO call maze routing
        if (!mazeRouting(twopin) )
            return false;
        return true;
    }
    twopin.segs = best_route;
    return true;
}

bool Solver::route2d(RouteInfo &info) {
    for (auto twopin : info.edges) 
        if (route2pin2d(*twopin) == false)
            return false;
    /// TODO update 2D congestion map
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

        for (int h = 1; h <= min_layer; ++h) {
            edge.dp_segs[i].cost[h] = 1e9;
            for (int k = 1; k <= min_layer; ++k) {
                /// (st.x, st.y, k) -> (en.x, en.y, k) -> (en.x, en.y, h)

                int dir = st.x == en.x ? 0 : 1;
                if ((k & 1) != dir) 
                /// layer with odd ID always be horizontal
                    continue;
                
                /// check congestion
                int con = get3DCon(Point(st.x, st.y, k), Point(en.x, en.y, k));
                con = std::min(con, get3DCon(Point(en.x, en.y, k), Point(en.x, en.y, h)));
                if (con <= 0)
                    continue;
                
                float prev_value = 0.0;
                if (i == 0)
                    prev_value = edge.child.dp.cost[k];
                else
                    prev_value = edge.dp_segs[i - 1].cost[k];
                float Zdist = h < k ? powerPrefixSum[k - 1] - powerPrefixSum[h - 1] : powerPrefixSum[h] - powerPrefixSum[k];
                float HVdist = (Point(st.x, st.y, k) - Point(en.x, en.y, k)).norm1() * Layers[k].powerFactor;
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
    node.dp.cost.resize(numLayer + 1, 0.0);
    node.dp.prev.resize(numLayer + 1);

    for (auto &edge : node.neighbor) {
        assign_layer_dp(edge.child, min_layer);
        edge_dp(edge, min_layer);
        for (int i = 1; i < numLayer; ++i)
            node.dp.cost[i] += edge.dp_segs.back().cost[i];
    }
    if (node.neighbor.size() == 0) {
        /// leaf node
        assert(node.isPin);
        int px = node.x, py = node.y;
        node.dp.cost.resize(numLayer + 1, F_INF);

        float cost = 0.0;
        for (int i = node.layer; i < numLayer; ++i) {
            if (get3DCon({px, py, node.layer}, {px, py, i}) <= 0)
                break;
            cost += Layers[i].powerFactor;
            node.dp.cost[i] = cost;
        }
        cost = 0.0;
        for (int i = node.layer; i >= 1; --i) {
            if (get3DCon({px, py, node.layer}, {px, py, i}) <= 0)
                break;
            cost += Layers[i].powerFactor;
            node.dp.cost[i] = cost;
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

        /// (st.x, st.y, next) -> (en.x, en.y, next) -> (en.x, en.y, cur)
        segs_3d.push_back(Segment({st.x, st.y, nextLayer}, {en.x, en.y, nextLayer}));
        segs_3d.push_back(Segment({en.x, en.y, nextLayer}, {en.x, en.y, curLayer}));
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
    node.layer = curLayer;
    for (auto &edge :node.neighbor) {
        int nextLayer = assign_layer_for_edge(edge, curLayer, segs_3d);
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