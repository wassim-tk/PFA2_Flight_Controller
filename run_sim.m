%% run_sim.m  --  build (if needed), simulate and plot
quad_params;
if ~exist('quad_sim.slx', 'file')
    build_quad_model;
end
out = sim('quad_sim');
plot_results(out);
animate_quad(out, P);
