#include "casino_roulette/casino_roulette.h"
#include "casino_roulette/casino_roulette_wheel_spinner.h"
#include "casino_roulette/casino_roulette_spin_commentator.h"

namespace casino_roulette
{

CasinoRoulette::CasinoRoulette(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
  , wheel_spinner_(new CasinoRouletteWheelSpinner(node_handle))
  , spin_commentator_(new CasinoRouletteSpinCommentator(node_handle))
{
  /* Здесь могла бы быть ваша реклама */
}

CasinoRoulette::~CasinoRoulette()
{
  delete wheel_spinner_;
  delete spin_commentator_;
}

void CasinoRoulette::start()
{
  wheel_spinner_->start();
}

} /* namespace casino_roulette */
