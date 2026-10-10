#include <iostream>
#include <vector>
#include <memory>
// #include <tuple>
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>

class State
{
public:
    typedef size_t StateID;
    State(StateID id) : id_(id) {}
    ~State(){}
    StateID getID() const { return id_; }
    double V_;
protected:
    StateID id_;
};

class Action
{
public:
    typedef size_t ActionID;
    Action(ActionID id) : id_(id) {}
    ~Action(){}
    ActionID getID() const { return id_; }
protected:
    ActionID id_;
};

class Outcome
{
public:
    typedef size_t OutcomeID;
    Outcome(OutcomeID id) : id_(id) {}
    ~Outcome(){}
    OutcomeID getID() const { return id_; }
protected:
    OutcomeID id_;
};

class Cluster
{
private:
public:
    Cluster()
    {

    }
    Cluster(const std::vector<std::shared_ptr<State>>& children)
    {
        children_ = children;
    }

    ~Cluster()
    {
    }
    std::vector<std::shared_ptr<State>> children_;
};
/*
namespace std
{
    template<>
    struct hash<State>
    {
            size_t operator()(const State& key)
            {
                return hash<State>()(key.)
            }
    }
}
*/
/*
class StateAction
{
public:
    typedef size_t StateActionID;
    StateAction(StateActionID id) : id_(id) {}
    ~StateAction(){}
    StateActionID getID() const { return id_; }
    std::vector<Cluster> Qp_;
protected:
    StateActionID id_;
};
*/


    typedef std::pair<State::StateID, Action::ActionID> saIDType;
    /// This structure hashes a state-action pair
    struct SAHash
    {
        // Hash function
        size_t operator()(const saIDType& saID) const
        {
            size_t hash = 0;
            boost::hash_combine(hash, saID.first);
            boost::hash_combine(hash, saID.second);
            return hash;
        }

        // Equality comparator for transitions
        bool operator()(const saIDType& lhs, const saIDType& rhs) const
        {
            return lhs.first == rhs.first &&
                   lhs.second == rhs.second;
        }
    };

void RDP()
{

}


int main()
{
    size_t N_states = 15*15*15*15;
    size_t N_actions = 16;
    size_t N_clusters = 4*4*4*4;
    size_t N_states_per_q = 54;
    State s(0);
    Action a(0);
    Outcome c(0);
    Cluster q;
    std::vector<std::shared_ptr<Cluster>> Qsa;
    saIDType saID;
    State::StateID sID = saID.first;
    std::cout << sID << std::endl;
    
    // Define state space
    
    std::vector<std::shared_ptr<State>> states;

    srand(time(0));
    for (size_t i = 0; i < N_states; i++)
    {
        states.push_back(std::make_shared<State>(i));
    }

    for (size_t j = 0; j < N_states_per_q; j++)
    {
        q.children_.push_back(states.at(j));
    }

    for (size_t ci = 0; ci < N_clusters; ci++)
    {
        Qsa.push_back(std::make_shared<Cluster>(q));
    }

    boost::unordered_map<saIDType, std::vector<std::shared_ptr<Cluster>> > saQMap;
    for (size_t i = 0; i < N_states; i++)
    {
        saID.first = i;
        for (size_t j = 0; j < N_actions; j++)
        {
            saID.second = j;
            saQMap[saID] = Qsa;
        }
    }

    std::cout << saQMap[saID].at(201)->children_.at(2)->getID() << std::endl;
    
    /*
    double V = 0.9;
    // 
*/
/*
    for (size_t i = 0; i < N_clusters; i++)
    {
        Q[i] = .push_back(q1);
        
        std::cout << sizeof(q1) << std::endl;
    // std::cout << q.at(i)->V_ << std::endl;
    }
*/
    return 0;
}