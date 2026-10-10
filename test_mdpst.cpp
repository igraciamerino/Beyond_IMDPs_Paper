#include <iostream>
#include <vector>
#include <memory>

class State
{
private:
public:
    State(double V)
    {
        V_ = V;
    }

    ~State()
    {
    }
    double V_;
};

int main()
{
    size_t N_states = 1e7;
    size_t N_clusters = 12 * 12 * 12 * 1 * 6 * 6 * 6 * 6;
    size_t N_states_per_q = 10;
    double V = 0.9;
    // std::vector<State> S;
    std::shared_ptr<State> s_ptr = std::make_shared<State>(V);
    std::vector<std::shared_ptr<State>> S, q, q1;
    std::vector<std::vector<std::shared_ptr<State>>> Q;
    srand(time(0));
    State s(0);
    for (size_t i = 0; i < N_states; i++)
    {
        S.push_back(std::make_shared<State>(i));
    }

    for (size_t j = 0; j < N_states_per_q; j++)
    {
        q1.push_back(S.at(j));
    }

    for (size_t i = 0; i < N_clusters; i++)
    {
        Q.push_back(q1);
        
        std::cout << sizeof(q1) << std::endl;
    // std::cout << q.at(i)->V_ << std::endl;
    }

    return 0;
}