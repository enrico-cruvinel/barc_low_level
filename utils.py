import matplotlib.pyplot as plt

def run(controller, model, state, ref, tf, dt):
    state_vec = []
    ref_vec = []

    while state.t < tf:
        controller.set_ref(ref)
        state_vec.append(state.copy())
        ref_vec.append(controller.ref)
        state.u_a = controller.step(state.v)
        model.step(state)
        state.t += dt

    return state_vec, ref_vec

def run_var_ref(controller, model, state, ref, tf, dt):
    state_vec = []
    ref_vec = []
    i = 0

    while state.t < tf:
        controller.set_ref(ref[i])
        state_vec.append(state.copy())
        ref_vec.append(controller.ref)
        state.u_a = controller.step(state.v)
        model.step(state)
        state.t += dt
        i += 1

    return state_vec, ref_vec

def plot_response(state_vec, ref_vec):
    
    t_vec = [state.t for state in state_vec]
    v_vec = [state.v for state in state_vec]
    u_a_vec = [state.u_a for state in state_vec]

    plt.figure(figsize = (6,6))

    plt.subplot(2,1,1)
    plt.plot(t_vec,v_vec,'b')
    plt.plot(t_vec,ref_vec,'k--')
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Velocity','Set Point'],loc='best')
    plt.title('System Response')

    plt.subplot(2,1,2)
    plt.plot(t_vec,u_a_vec,'r')
    plt.ylabel('Input (PWM)')    
    plt.legend(['Input'])
    plt.xlabel('Time (sec)')

    plt.show()
