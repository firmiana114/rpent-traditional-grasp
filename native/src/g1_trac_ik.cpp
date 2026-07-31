#include <trac_ik/trac_ik.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct ChainDescription
{
  KDL::Chain chain;
  KDL::JntArray lower;
  KDL::JntArray upper;
  std::vector<std::string> joint_names;
};

double read_double(std::istringstream& input, const std::string& context)
{
  double value = 0.0;
  if (!(input >> value) || !std::isfinite(value))
    throw std::runtime_error("invalid number for " + context);
  return value;
}

KDL::Frame read_origin(std::istringstream& input, const std::string& name)
{
  const double x = read_double(input, name + ".x");
  const double y = read_double(input, name + ".y");
  const double z = read_double(input, name + ".z");
  const double roll = read_double(input, name + ".roll");
  const double pitch = read_double(input, name + ".pitch");
  const double yaw = read_double(input, name + ".yaw");
  return KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));
}

ChainDescription load_chain(const std::string& path)
{
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("cannot open chain file: " + path);

  std::string line;
  if (!std::getline(file, line) || line != "RPENT_G1_CHAIN_V1")
    throw std::runtime_error("unsupported chain format");

  KDL::Chain chain;
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<std::string> names;
  bool ended = false;
  while (std::getline(file, line))
  {
    if (line.empty())
      continue;
    std::istringstream input(line);
    std::string type;
    input >> type;
    if (type == "BASE" || type == "ARM")
      continue;
    if (type == "END")
    {
      ended = true;
      break;
    }
    std::string joint_name;
    std::string child_link;
    if (!(input >> joint_name >> child_link))
      throw std::runtime_error("invalid chain line: " + line);
    const KDL::Frame origin = read_origin(input, joint_name);
    if (type == "FIXED")
    {
      chain.addSegment(KDL::Segment(
        child_link, KDL::Joint(joint_name, KDL::Joint::None), origin));
      continue;
    }
    if (type != "REVOLUTE")
      throw std::runtime_error("unknown chain record: " + type);
    const KDL::Vector axis(
      read_double(input, joint_name + ".axis_x"),
      read_double(input, joint_name + ".axis_y"),
      read_double(input, joint_name + ".axis_z"));
    const double lower_limit = read_double(input, joint_name + ".lower");
    const double upper_limit = read_double(input, joint_name + ".upper");
    const KDL::Joint joint(
      joint_name,
      origin.p,
      origin.M * axis,
      KDL::Joint::RotAxis);
    chain.addSegment(KDL::Segment(child_link, joint, origin));
    names.push_back(joint_name);
    lower.push_back(lower_limit);
    upper.push_back(upper_limit);
  }
  if (!ended || names.size() != 7 || chain.getNrOfJoints() != 7)
    throw std::runtime_error("chain must end with exactly seven movable joints");

  ChainDescription result{chain, KDL::JntArray(7), KDL::JntArray(7), names};
  for (std::size_t index = 0; index < 7; ++index)
  {
    result.lower(index) = lower[index];
    result.upper(index) = upper[index];
  }
  return result;
}

KDL::JntArray read_joints(std::istringstream& input)
{
  KDL::JntArray joints(7);
  for (std::size_t index = 0; index < 7; ++index)
    joints(index) = read_double(input, "joint");
  return joints;
}

KDL::Frame read_frame(std::istringstream& input)
{
  const KDL::Vector position(
    read_double(input, "target.x"),
    read_double(input, "target.y"),
    read_double(input, "target.z"));
  KDL::Rotation rotation;
  for (std::size_t row = 0; row < 3; ++row)
    for (std::size_t column = 0; column < 3; ++column)
      rotation(row, column) = read_double(input, "target.rotation");
  return KDL::Frame(rotation, position);
}

void write_joints(const KDL::JntArray& joints)
{
  std::cout << "OK";
  for (std::size_t index = 0; index < 7; ++index)
    std::cout << ' ' << joints(index);
  std::cout << '\n' << std::flush;
}

void write_frame(const KDL::Frame& frame)
{
  std::cout << "OK " << frame.p.x() << ' ' << frame.p.y() << ' ' << frame.p.z();
  for (std::size_t row = 0; row < 3; ++row)
    for (std::size_t column = 0; column < 3; ++column)
      std::cout << ' ' << frame.M(row, column);
  std::cout << '\n' << std::flush;
}

KDL::Frame forward(
  KDL::ChainFkSolverPos_recursive& solver,
  const KDL::JntArray& joints)
{
  KDL::Frame frame;
  if (solver.JntToCart(joints, frame) < 0)
    throw std::runtime_error("forward kinematics failed");
  return frame;
}

double rotation_error(const KDL::Rotation& first, const KDL::Rotation& second)
{
  KDL::Vector axis;
  return (first.Inverse() * second).GetRotAngle(axis);
}

int self_test(const ChainDescription& description)
{
  KDL::ChainFkSolverPos_recursive fk(description.chain);
  KDL::JntArray expected(7);
  const std::array<double, 7> values{0.2, -0.15, 0.1, 0.45, -0.1, 0.2, 0.05};
  for (std::size_t index = 0; index < values.size(); ++index)
    expected(index) = values[index];
  const KDL::Frame target = forward(fk, expected);
  KDL::JntArray seed(7);
  KDL::JntArray solved(7);
  TRAC_IK::TRAC_IK solver(
    description.chain,
    description.lower,
    description.upper,
    0.1,
    1e-6,
    TRAC_IK::Distance);
  if (solver.CartToJnt(seed, target, solved) < 0)
    throw std::runtime_error("self-test IK did not converge");
  const KDL::Frame actual = forward(fk, solved);
  const double position_residual = (target.p - actual.p).Norm();
  const double orientation_residual = rotation_error(target.M, actual.M);
  if (position_residual > 1e-4 || orientation_residual > 1e-3)
    throw std::runtime_error("self-test FK residual exceeded tolerance");
  std::cerr << "[g1_trac_ik] self-test passed: position_residual="
            << position_residual << " orientation_residual="
            << orientation_residual << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 3 && argc != 4)
  {
    std::cerr
      << "usage: g1_trac_ik CHAIN_FILE [--self-test | TIMEOUT TOLERANCE]"
      << std::endl;
    return 2;
  }
  try
  {
    std::cout << std::setprecision(17);
    const ChainDescription description = load_chain(argv[1]);
    if (argc == 3)
    {
      if (std::string(argv[2]) != "--self-test")
        throw std::runtime_error("unknown option");
      return self_test(description);
    }
    KDL::ChainFkSolverPos_recursive fk(description.chain);
    const double timeout = argc == 4 ? std::stod(argv[2]) : 0.02;
    const double tolerance = argc == 4 ? std::stod(argv[3]) : 1e-5;
    if (!(timeout > 0.0) || !(tolerance > 0.0))
      throw std::runtime_error("timeout and tolerance must be positive");
    TRAC_IK::TRAC_IK solver(
      description.chain,
      description.lower,
      description.upper,
      timeout,
      tolerance,
      TRAC_IK::Distance);
    std::cout << "READY 7\n" << std::flush;
    std::string line;
    while (std::getline(std::cin, line))
    {
      try
      {
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command == "QUIT")
          break;
        if (command == "FK")
        {
          write_frame(forward(fk, read_joints(input)));
          continue;
        }
        if (command == "IK")
        {
          const KDL::JntArray seed = read_joints(input);
          const KDL::Frame target = read_frame(input);
          KDL::JntArray solution(7);
          if (solver.CartToJnt(seed, target, solution) < 0)
            throw std::runtime_error("no IK solution");
          write_joints(solution);
          continue;
        }
        throw std::runtime_error("unknown command");
      }
      catch (const std::exception& error)
      {
        std::cout << "ERR " << error.what() << '\n' << std::flush;
      }
    }
  }
  catch (const std::exception& error)
  {
    std::cerr << "[g1_trac_ik][fatal] " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
