for i = 1:242
    
    inputData{i,1} = readXYZ(msgs{i,1});
end

save('inputData.mat', 'inputData');