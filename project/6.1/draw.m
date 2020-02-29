function draw = draw()

    draw=0;

    quad = Quad(); 
    CTRL = ctrl_NMPC(quad);
    sim = quad.sim(CTRL);
    quad.plot(sim);



end