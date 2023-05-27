function A = isfree(qr,CollisionArray,env4)
    tmp = 1;
    max_r = 0;
    for r = 0:.1:pi
        out = 0;
        env4{1}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 100-100]);
        env4{2}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 100-100]);
        env4{3}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 200-100]);
        env4{4}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 243-100]);
        env4{5}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 268-100]);
        env4{6}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 320-100]);
        env4{7}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 355-100]);
        env4{8}.Pose = trvec2tform(qr)*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -r])*trvec2tform([0 0 (170-100)]);     
        for i = 1:1:length(CollisionArray)
            for j = 1:1:length(env4)
                out = checkCollision(env4{j},CollisionArray{i});
                if out == 1
%                     i
%                     j
                    break;
                end
            end
            if out == 1
                break;
            end
        end
        if out == 0
            max_r = r;
            tmp = 0;
        end
    end
    A = [tmp,max_r];

end