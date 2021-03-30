function getQdesSV(state,qdds::Array{Float64})
    # Creates the SegmentedVector structure using the state
    qdd_des = similar(velocity(state))
    for i in 1:size(qdds)[1]
      qdd_des[i] = qdds[i];
    end
    return qdd_des
end

function solve_quintic(t_0::Float64, t_f::Float64, b::Vector{Float64})
    A = [1 t_0 t_0^2 t_0^3 t_0^4 t_0^5;
         0 1 2*t_0 3*t_0^2 4*t_0^3 5*t_0^4;
         0 0 2 6*t_0 12*t_0^2 20*t_0^3; 
         1 t_f t_f^2 t_f^3 t_f^4 t_f^5;
         0 1 2*t_f 3*t_f^2 4*t_0^3 5*t_f^4;
         0 0 2 6*t_f 12*t_f^2 20*t_f^3]
    x = inv(A)*b
    return x
end

function traj(t::Float64)
    # implements a quintic polynomial function to solve the joints at a given point in time
    # compute the desired joint angle at time t
    q_0 = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    q_f = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
    v_0 = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    v_f = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    a_0 = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    a_f = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    t_0 = 0.0
    t_f = 5.0

    q_of_t = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    for i in 1:size(q_f)[1]
      as = solve_quintic(t_0, t_f, [q_0[i]; v_0[i]; a_0[i]; q_f[i]; v_f[i]; a_f[i]])
      q_of_t[i] = as[1] + as[2]*t + as[3]*t^2 + as[4]*t^3 + as[5]*t^4 + as[6]*t^5
    end
    return q_of_t
end


function control_PD!(τ, t, state)
    # Compute a value for τ
    Kd = 40
    Kp = 300
    goal = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
    τ .= - Kd .* velocity(state) - Kp*(configuration(state) - goal)
    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

function control_CTC!(τ, t, state)
    # Compute a value for τ
    q_des = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
    v_des = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    a_des = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0]
    Kd = 20
    Kp = 50
    # q_des = [1.0,-1.0,2.0]
    # v_des = [0.0,0.0,0.0]
    # a_des = [0.0,0.0,0.0]
    e =  configuration(state) - q_des
    # Inverse dynamics Control equation for acceleration (4.27)
    aq = a_des .- Kp * e .- Kd .* (velocity(state) - v_des)
    
    # plug those accelerations into the model's torques
    τ .= inverse_dynamics(state, getQdesSV(state, aq))

    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

function update_state(state,mvis,qs::Array{Float64})
    set_configuration!(state, qs)
    set_configuration!(mvis, configuration(state))
end

function move_robot_PD(mvis,mechanism)
    state=MechanismState(mechanism)
    update_state(state,mvis,[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    problem = ODEProblem(Dynamics(mechanism,control_PD!), state, (0., 5.))
    sol = solve(problem, Vern7())
    setanimation!(mvis, sol; realtime_rate = 1.0);

    ### do some plotting
    return sol
end

function move_robot_CTC(mvis,mechanism)
    state=MechanismState(mechanism)
    update_state(state,mvis,[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    problem = ODEProblem(Dynamics(mechanism,control_CTC!), state, (0., 5.))
    sol = solve(problem, Vern7())
    setanimation!(mvis, sol; realtime_rate = 1.0);

    ### do some plotting
    return sol
end

function plot_sol(p,sol,colorarg,saveflag,savename)
    qsol = vcat(sol[:]'...)
    for i=1:3
        push!(p,layer(x=sol.t,y=qsol[:,i],Geom.line,color=colorarg))
    end
    p
    if saveflag
        p |> PDF(savename)
    end
end


goal = [0.01,-0.5,-0.0,-2.0,-0.3,1.5,-0.7,0.1,0.1]
## to restart visualization:
delete!(vis)
mvis, mechanism = display_urdf("panda.urdf",vis)
state = MechanismState(mechanism)
set_configuration!(state,[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5])
set_configuration!(mvis, configuration(state))
set_velocity!(state,[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0])
qdd = getQdesSV(state,[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0])
inverse_dynamics(state,qdd)
# pick move_robot_CTC or move_robot_PD here
sol = move_robot_PD(mvis,mechanism);
p = plot()
plot_sol(p,sol,[colorant"gold"],false,"foo.pdf")
println("final state: ", sol[end])
println("error: ", sol[end][1:9] - goal)
p
