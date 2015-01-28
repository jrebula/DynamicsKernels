function [] = rng(seed)
% rng seeds the current random number stream 

stream = RandStream('mt19937ar', 'Seed', seed);
RandStream.setDefaultStream(stream);

end

