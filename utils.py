def run(controller, model, state, tf, dt):
    state_vec = []
    ref_vec = []

    while state.t < tf:
        controller.set_ref(10)
        state_vec.append(state.copy())
        ref_vec.append(controller.ref)
        state.u = controller.step(state.v)
        model.step(state)
        state.t += dt

    return state_vec, ref_vec