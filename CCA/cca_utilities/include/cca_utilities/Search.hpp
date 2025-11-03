#pragma once
#include <map>
#include <vector>
#include <list>
#include <functional>

//Interface
template<typename NodeType, typename CostType>
class Searchable
{
    public:
     virtual std::vector<std::pair<NodeType,CostType>> edgesFrom(NodeType node) = 0;
     virtual CostType getNul() = 0;
     virtual CostType getHzn() = 0;
     virtual CostType callHrst(NodeType node, NodeType end) = 0;
};

//Class
class Search
{
    public:
        template<typename NodeType, typename CostType>
        static std::pair<CostType,std::vector<NodeType>> A_star(NodeType start, NodeType end, Searchable<NodeType, CostType> * graph);

        template<typename NodeType, typename CostType, typename terminate>
        static std::pair<CostType,std::vector<NodeType>> Dijkstra(NodeType start, Searchable<NodeType, CostType> * graph, terminate term);
};

template<typename NodeType, typename CostType>
std::pair<CostType,std::vector<NodeType>> Search::A_star(NodeType start, NodeType end, Searchable<NodeType, CostType> * graph)
{
    //priority queue init
    std::list<NodeType> search_list;
    search_list.push_back(start);

    //records init
    std::map<NodeType,std::pair<CostType, std::vector<NodeType>>> paths;
    std::vector<NodeType> s({start});
    std::pair<CostType,std::vector<NodeType>> pr(graph->getNul(), s);
    paths[start] = pr;

    std::vector<std::pair<NodeType,CostType>> next_steps;
    while(*search_list.begin() != end)//exit if end
    {
        next_steps = graph->edgesFrom(*search_list.begin());//get accessible edges

        for (auto step: next_steps)//evaluate each step
        {
            if (paths.find(step.first) == paths.end())//if vertex is new add
            {
                search_list.push_back(step.first);
                std::vector<NodeType> s;
                s = paths[*search_list.begin()].second;
                s.push_back(step.first);
                std::pair<CostType,std::vector<NodeType>> pr(step.second + paths[*search_list.begin()].first, s);
                paths[step.first] = pr;
            }else{//update cost to go
                if (paths[step.first].first > paths[*search_list.begin()].first + step.second)
                {
                    std::vector<NodeType> s;
                    s = paths[*search_list.begin()].second;
                    s.push_back(step.first);
                    std::pair<CostType,std::vector<NodeType>> pr(step.second + paths[*search_list.begin()].first, s);
                    paths[step.first] = pr;
                }
                
            }
            
        }

        //remove visited vertex
        search_list.pop_front();
        if (search_list.size() < 1)//check failure condition
        {
            std::vector<NodeType> s;
            std::pair<CostType,std::vector<NodeType>> no_solution(graph->getHzn(), s);
            return no_solution;
        }

        //combine heuristic
        std::list<std::pair<CostType, NodeType>> queue;
        std::pair<CostType,NodeType> cost;
        for (auto search : search_list)
        {
            cost.first = paths[search].first + graph->callHrst(search,end);
            cost.second = search;
            queue.push_back(cost);
        }

        //sort list
        queue.sort();
        search_list.clear();
        for (auto q : queue)
        {
            search_list.push_back(q.second);
        }
    }

    return paths[end];
}

template<typename NodeType, typename CostType, typename terminate>
std::pair<CostType,std::vector<NodeType>> Search::Dijkstra(NodeType start, Searchable<NodeType, CostType> * graph, terminate term)
{
    //priority queue init
    std::list<NodeType> search_list;
    search_list.push_back(start);

    //records init
    std::map<NodeType,std::pair<CostType, std::vector<NodeType>>> paths;
    std::vector<NodeType> s({start});
    std::pair<CostType,std::vector<NodeType>> pr(graph->getNul(), s);
    paths[start] = pr;

    std::vector<std::pair<NodeType,CostType>> next_steps;
    while(!term(*search_list.begin())){//exit if no nodes left
        
        next_steps = graph->edgesFrom(*search_list.begin());//get accessible edges

        for (auto step: next_steps)//evaluate each step
        {
            if (paths.find(step.first) == paths.end())//if vertex is new add
            {
                search_list.push_back(step.first);
                std::vector<NodeType> s;
                s = paths[*search_list.begin()].second;
                s.push_back(step.first);
                std::pair<CostType,std::vector<NodeType>> pr(step.second + paths[*search_list.begin()].first, s);
                paths[step.first] = pr;
            }else{//update cost to go
                if (paths[step.first].first > paths[*search_list.begin()].first + step.second)
                {
                    std::vector<NodeType> s;
                    s = paths[*search_list.begin()].second;
                    s.push_back(step.first);
                    std::pair<CostType,std::vector<NodeType>> pr(step.second + paths[*search_list.begin()].first, s);
                    paths[step.first] = pr;
                }
                
            }
            
        }

        //remove visited vertex
        search_list.pop_front();
        if (search_list.size() < 1)//check failure condition
        {
            std::vector<NodeType> s;
            std::pair<CostType,std::vector<NodeType>> no_solution(graph->getHzn(), s);
            return no_solution;
        }

        //combine heuristic
        std::list<std::pair<CostType, NodeType>> queue;
        for (auto search : search_list)
        {
            std::pair<CostType,NodeType> cost(paths[search].first, search);
            queue.push_back(cost);
        }

        //sort list
        queue.sort();
        search_list.clear();
        for (auto q : queue){search_list.push_back(q.second);}
    }

    return paths[*search_list.begin()];
}