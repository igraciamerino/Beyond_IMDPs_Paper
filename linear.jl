
using LinearAlgebra,JuMP, HiGHS
using MAT

lp_matrices = matread("lp_matrices.mat")
f = lp_matrices["f"]
A = lp_matrices["A"]
b = lp_matrices["b"]
Aeq = lp_matrices["Aeq"]
beq = lp_matrices["beq"]

Aeq = vec(Aeq)
f = vec(f)

n = length(f)
lb = zeros(n)

# Optimization function
function linear_program(f, A, b, Aeq, beq, lb)

    # Using HiGHS as the LP solver
    model = Model(HiGHS.Optimizer)
    set_silent(model)

    # Create optimization variables
    n = length(f)
    @variable(model, x[1:n])
    set_lower_bound.(x, lb)
    #@variable(model, x[1:n], lower_bound=zeros(n), upper_bound=1)   

    # Define constraints
    @constraint(model, A*x .<= b)
    @constraint(model, Aeq'*x == beq)

    # Construct barriers
    @objective(model, Min, f'*x)

    # Optimize model
    JuMP.optimize!(model)

    # Barrier certificate
    x = value.(x)

    # Print model summary
    println("")
    println(solution_summary(model))
    # Note: comment this out for more speed

    return x
end

linear_program(f, A, b, Aeq, beq, lb)