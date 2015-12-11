#include "idmind_behaviours/idmind_behaviours.h"

bool IdmindBehaviours::readFiles()
{
  std::string homedir = getenv("HOME");
  std::string folder = homedir + behaviours_folder_;

  std::vector<std::vector<int> > my_c;
  readNumbersFile("colours.txt", folder.c_str(), &my_c);

  for (int i=0; i<my_c.size(); i++)
  {
    Colour c;
    c.red = my_c[i][0];
    c.green = my_c[i][1];
    c.blue = my_c[i][2];

    colours_.push_back(c);
  }

  std::vector<std::vector<int> > my_b;
  readNumbersFile("behaviours.txt", folder.c_str(), &my_b);

  for (int i=0; i<my_b.size(); i++)
  {
    Behaviour b;
    b.head = my_b[i][0];
    b.left_arm = my_b[i][1];
    b.right_arm = my_b[i][2];
    b.mouth = my_b[i][3];

    for (int j=4; j<10; j++)
      b.leds.push_back(my_b[i][j]);

    behaviours_.push_back(b);
  }

  return true;
}

template<typename T>
bool IdmindBehaviours::readNumbersFile(std::string file, std::string folder, std::vector<std::vector<T> >* vector)
{
  std::ifstream my_file((folder + file).c_str());

  if (!my_file)
    ROS_ERROR("%s --> Failed to open %s file.", ros::this_node::getName().c_str(), file.c_str());

  std::string my_line;
  std::vector<T> row;

  int i = 0;
  while (std::getline(my_file, my_line))
  {
    int comment_pos = my_line.find('#');
    if (comment_pos >= 0)
      my_line.erase(comment_pos);

    if (my_line.size() != 0)
    {
      (*vector).push_back(row);
      std::vector<int> spaces;

      for (int k=0; k<my_line.size(); k++)
        if (my_line[k] == '\t')
          spaces.push_back(k);

      for (int j=0; j<spaces.size()-1; j++)
        (*vector)[i].push_back(atof(my_line.substr(spaces[j]+1, spaces[j+1]-spaces[j]-1).c_str()));

      (*vector)[i].push_back(atof(my_line.substr(spaces[spaces.size()-1]+1).c_str()));

      i++;
    }
  }

  my_file.close();
  return true;
}
