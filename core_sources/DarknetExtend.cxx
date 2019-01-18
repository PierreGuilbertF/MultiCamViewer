/*=========================================================================

  Program:   Visualization Toolkit
  Module:    DarknetExtend.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @brief   Extend some function to manipulate darknet
 *
 * Calibrator is a convenience class for calibrating two or more camera.
 * It gives the intrinsecs and extrinsecs parameters of each camera relatively
 * to the first camera's camera frames
 *
*/

#include "DarknetExtend.h"

//---------------------------------------------------------------------------
std::pair<std::vector<BoundedBox>, std::vector<BoundedBox> > LaunchDarknetAndGetROI(std::string filename1, std::string filename2, std::string weightFilename,
                            std::string yoloConfigFilename, std::string yoloConfigFilename2, std::string yoloLabelsFilename, double thresh)
{
  ////// YOLO INITIALIZATION ///////
  srand(222222);

  // get class labels
  list *options = read_data_cfg(const_cast<char *>(yoloConfigFilename2.c_str()));
  char **names = get_labels(const_cast<char *>(yoloLabelsFilename.c_str()));
  image **alphabet = load_alphabet();

  // Load the YOLO neuronal network structure
  network net = parse_network_cfg(const_cast<char *>(yoloConfigFilename.c_str()));

  // Once the structure is loaded, load the weight of the network
  load_weights(&net, const_cast<char *>(weightFilename.c_str()));

  // Set the batch to 1. The batch represent the subdivision of the image
  set_batch_network(&net, 1);

  // load the images in YOLO format
  image yoloIm1 = load_image_color(const_cast<char *>(filename1.c_str()), 0, 0);
  image yoloIm2 = load_image_color(const_cast<char *>(filename2.c_str()), 0, 0);

  // Now we will resized the image according to the network training
  image yoloIm1Sized = letterbox_image(yoloIm1, net.w, net.h);
  image yoloIm2Sized = letterbox_image(yoloIm2, net.w, net.h);

  // Get the last layer of the network
  layer l = net.layers[net.n-1];

  /////////// PROCESSING ////////
  // Buffer of boxes ?
  box *boxes = reinterpret_cast<box *>(calloc(l.w * l.h * l.n, sizeof(box)));

  // Results of the classification. It is the probabilities of each
  // classes according to the network in the corresponding box. The image
  // is splitted in boxes (depending on the batch number). Each boxes output
  // is the probabilities to belong to the classes
  float **probs = reinterpret_cast<float **>(calloc(l.w * l.h * l.n, sizeof(float *)));
  for(int j = 0; j < l.w * l.h * l.n; ++j)
  {
    probs[j] = reinterpret_cast<float *>(calloc(l.classes + 1, sizeof(float *)));
  }

  // mask of what ?
  float **masks = 0;

  if (l.coords > 4)
  {
    masks = reinterpret_cast<float **>(calloc(l.w * l.h * l.n, sizeof(float*)));
    for(int j = 0; j < l.w * l.h * l.n; ++j)
    {
      masks[j] = reinterpret_cast<float *>(calloc(l.coords - 4, sizeof(float *)));
    }
  }

  // Input vector (it is the vectorialized input image)
  float *X1 = yoloIm1Sized.data;
  float *X2 = yoloIm2Sized.data;
  double time = what_time_is_it_now();

  // Compute the classification using the network
  network_predict(net, X1);

  printf("%s: Predicted in %f seconds.\n", const_cast<char *>(filename1.c_str()), what_time_is_it_now() - time);

  // Get the zone of interest
  float hier_thresh = 0.5;
  get_region_boxes(l, yoloIm1.w, yoloIm1.h, net.w, net.h, thresh, probs, boxes, masks, 0, 0, hier_thresh, 1);
  float nms = 0.3;
  if(nms)
  {
    do_nms_obj(boxes, probs, l.w * l.h * l.n, l.classes, nms);
  }
  std::vector<BoundedBox> Res1 = GetRegionOfInterestYolo(yoloIm1, l.w * l.h * l.n, thresh, boxes, probs, masks, names, alphabet, l.classes);

  // Compute the classification using the network
  time = what_time_is_it_now();
  network_predict(net, X2);

  printf("%s: Predicted in %f seconds.\n", const_cast<char *>(filename2.c_str()), what_time_is_it_now() - time);

  // Get the zone of interest
  get_region_boxes(l, yoloIm2.w, yoloIm2.h, net.w, net.h, thresh, probs, boxes, masks, 0, 0, hier_thresh, 1);
  if(nms)
  {
    do_nms_obj(boxes, probs, l.w * l.h * l.n, l.classes, nms);
  }
  std::vector<BoundedBox> Res2 = GetRegionOfInterestYolo(yoloIm2, l.w * l.h * l.n, thresh, boxes, probs, masks, names, alphabet, l.classes);
  /*draw_detections(yoloIm2, l.w*l.h*l.n, thresh, boxes, probs, masks, names, alphabet, l.classes);
  save_image(yoloIm2, "predictions");*/

  std::pair<std::vector<BoundedBox>, std::vector<BoundedBox> > Res(Res1, Res2);
  return Res;
}

//---------------------------------------------------------------------------
std::vector<BoundedBox> GetRegionOfInterestYolo(image im, int num, float thresh, box *boxes, float **probs, float **masks, char **names, image **alphabet, int classes)
{
  std::vector<BoundedBox> Res;

  // loop over the batch area
  for(int i = 0; i < num; ++i)
  {
    // get the higher probability label
    int label = max_index(probs[i], classes);
    float prob = probs[i][label];

    // if the probability is higher than the threshold
    if(prob > thresh)
    {
      // What means this 0.006 ? Who knows....
      int width = im.h * .006;

      // print the label and the corresponding probability
      printf("%s: %.0f%%\n", names[label], prob * 100);

      box b = boxes[i];

      BoundedBox temp;

      // Get the bounds
      temp.Left  = (b.x - b.w / 2.) * im.w;
      temp.Right = (b.x + b.w / 2.) * im.w;
      temp.Top   = (b.y - b.h / 2.) * im.h;
      temp.Bot   = (b.y + b.h / 2.) * im.h;

      // Clamp the bounds
      if(temp.Left < 0) temp.Left = 0;
      if(temp.Right > im.w - 1) temp.Right = im.w - 1;
      if(temp.Top < 0) temp.Top = 0;
      if(temp.Bot > im.h-1) temp.Bot = im.h - 1;

      Res.push_back(temp);
    }
  }

  return Res;
}
