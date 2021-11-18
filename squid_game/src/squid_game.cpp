#include "squid_game/squid_game.h"
#include "squid_game/squid_game_glass_bridge.h"
#include "squid_game/squid_game_player.h"

namespace squid_game
{

SquidGame::SquidGame(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
  , glass_bridge_(new SquidGameGlassBridge(node_handle_))
  , player_(new SquidGamePlayer(node_handle_))
{
  /* Здесь могла бы быть ваша реклама */
}

SquidGame::~SquidGame()
{
  delete glass_bridge_;
  delete player_;
}

void SquidGame::start()
{
  glass_bridge_->start();
}

} /* namespace squid_game */
