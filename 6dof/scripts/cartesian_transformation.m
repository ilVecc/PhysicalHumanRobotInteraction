function Xq = cartesian_transformation(dh, qq)

    for i=1:length(qq)
        Xq(i,:)= K(dh, qq(i,:));
    end

end