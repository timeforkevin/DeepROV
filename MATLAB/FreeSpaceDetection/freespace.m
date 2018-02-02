% clear all;

hBarWidth = 95;
vBarWidth = 5;
downsampleFactor = 4;

v = VideoReader('mov1.mp4');
vidWidth = v.Width;
vidHeight = v.Height;
newVidWidth = vidWidth-vBarWidth*2+1;
newVidHeight = vidHeight-hBarWidth*2+1;
mov = struct('cdata',zeros(newVidHeight,newVidWidth,3,'uint8'),'colormap',autumn(255));
k = 1;
while hasFrame(v)
% while k < 10
    f = readFrame(v);
    f = imresize(f(hBarWidth:end-hBarWidth, vBarWidth:end-vBarWidth, :), 1/downsampleFactor);
    downHeight = size(f, 1);
    downWidth = size(f, 2);
    
    fgray = rgb2gray(f);
    SE = strel('disk', 7);
    filtered = double(stdfilt(fgray, SE.Neighborhood));
%     smooth = filtered < 5;
    smooth = filtered/max(max(filtered));
    I = im2uint8(smooth);
%     fb(1:newVidHeight,:,:) = repmat(I, 1, 1, 3);
%     fb(newVidHeight+1:end,:,:) = f;
    fb = zeros(downHeight,downWidth*2,3,'uint8');
    fb(:,1:downWidth,:) = repmat(I, 1, 1, 3);
    fb(:,downWidth+1:end,:) = f;
    mov(k).cdata = fb;
    k = k+1;
end
hf = figure(1);
% set(hf,'position',[0 0 downWidth*2 downHeight]);
movie(hf,mov,1,v.FrameRate);