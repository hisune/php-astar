<?php
/**
 * Created by Hisune
 * Designed by Hisune.com
 * User: Hisune
 * Date: 14-12-1
 * Time: 下午2:10
 * Email: hi@hisune.com
 */
 
$config = array(
    'start' => array(1, 2), // 开始坐标
    'end' => array(9, 10), // 结束坐标
    'x' => 10, // 最大x
    'y' => 10, // 最大y
    'disable_num' => 30, // 障碍点个数
);
 
$a = new aStar($config['start'], $config['end'], $config['x'], $config['y'], $config['disable_num']);
$a->displayPic();
 
 
/**
 * astar寻路算法
 */
class aStar
{
 
    private $_start; // 开始点
    private $_end; // 结束点
    private $_x; // 最大x轴
    private $_y; // 最大y轴
    private $_num; // 障碍点数量
 
    private $_around; // 当前节点的可能四周节点数组
    private $_g; // g值数组
 
    public $open; // 开放节点数组
    public $close; // 关闭节点数组
    public $disable = array(); // 随机生成的障碍点数组
 
    public $route = array(); // 结果路径数组
 
    /**
     * @param $start array 开始点
     * @param $end array 结束点
     * @param $x int 最大x轴
     * @param $y int 最大y轴
     * @param $num int 最大随机障碍点数量
     */
    public function __construct($start, $end, $x, $y, $num)
    {
        $this->_start = $start;
        $this->_end = $end;
        $this->_x = $x;
        $this->_y = $y;
        $this->_num = $num;
 
        // 开始寻路
        $this->_route();
    }
 
    private function _route()
    {
        // 生成随机路障点
        $this->_makeDisable();
        // 算法开始
        $this->_start();
    }
 
    private function _start()
    {
        // 设置初始点的各项值
        $point[0] = $this->_start[0]; // x
        $point[1] = $this->_start[1]; // y
        $point['i'] = $this->_pointInfo($this->_start); // 当前节点 point info
        $point['f'] = 0; // f 值
        $this->_g[$point['i']] = 0; // g 值
        $point['h'] = $this->_getH($this->_start); // h 值
        $point['p'] = null; // 父节点 point info
 
        $this->open[$point['i']] = $this->close[$point['i']] = $point; // 开始节点加入open和close
        while (count($this->open) > 0) {
            // 查找最小的f值
            $f = 0;
            foreach ($this->open as $info => $node) {
                if ($f === 0 || $f > $node['f']) {
                    $minInfo = $info;
                    $f = $node['f'];
                }
            }
 
            // 将当前节点从open中删除
            $current = $this->open[$minInfo];
            unset($this->open[$minInfo]);
            // 将当前节点加入close
            $this->close[$minInfo] = $current;
 
            // 如果到达了终点，根据各节点的父节点算出其route
            if ($current[0] == $this->_end[0] && $current[1] == $this->_end[1]) {
                // 反向推出路径
                while ($current['p'] !== null) {
                    $tmp = $this->close[$this->_pointInfo($current['p'])];
                    array_unshift($this->route, array($tmp[0], $tmp[1]));
                    $current = $this->close[$this->_pointInfo($current['p'])];
                }
                array_push($this->route, $this->_end);
                break;
            }
 
            // 设置当前节点的四周节点
            $this->_setAround($current);
            // 四周节点状态更新
            $this->_updateAround($current);
        }
 
    }
 
    private function _updateAround($current)
    {
        foreach ($this->_around as $v) {
            if (!isset($this->close[$this->_pointInfo($v)])) { // 不在close里面才处理
                if (isset($this->open[$this->_pointInfo($v)])) { // 在open里面，比较值，小则更新
                    if ($this->_getG($current) < $this->_g[$this->_pointInfo($v)]) {
                        $this->_updatePointDetail($current, $v);
                    }
                } else { // 不在open里面，直接更新
                    $this->open[$this->_pointInfo($v)][0] = $v[0];
                    $this->open[$this->_pointInfo($v)][1] = $v[1];
                    $this->_updatePointDetail($current, $v);
                }
            }
        }
    }
 
    private function _updatePointDetail($current, $around)
    {
        $this->open[$this->_pointInfo($around)]['f'] = $this->_getF($current, $around);
        $this->_g[$this->_pointInfo($around)] = $this->_getG($current);
        $this->open[$this->_pointInfo($around)]['h'] = $this->_getH($around);
        $this->open[$this->_pointInfo($around)]['p'] = $current; // 重新设置父节点
    }
 
    /**
     * 返回当前节点的可能四周节点
     */
    private function _setAround($point)
    {
        // 可能的X点
        $roundX[] = $point[0]; // 当前x点
        ($point[0] - 1 > 0) && $roundX[] = $point[0] - 1; // 不越界（最小）
        ($point[0] + 1 <= $this->_x) && $roundX[] = $point[0] + 1; // // 不越界（最大）
        // 可能的Y点
        $roundY[] = $point[1];
        ($point[1] - 1 > 0) && $roundY[] = $point[1] - 1;
        ($point[1] + 1 <= $this->_y) && $roundY[] = $point[1] + 1;
 
        $this->_around = array();
        foreach ($roundX as $vX) {
            foreach ($roundY as $vY) {
                $tmp = array(
                    0 => $vX,
                    1 => $vY,
                );
                // 不在障碍点内, 不在关闭节点内，不是他本身, 不是对角线
                if (
                    !in_array($tmp, $this->disable) &&
                    !in_array($tmp, $this->close) &&
                    !($vX == $point[0] && $vY == $point[1]) &&
                    ($vX == $point[0] || $vY == $point[1])
                )
                    $this->_around[] = $tmp;
            }
        }
    }
 
    /**
     * 返回当前节点的唯一key
     */
    private function _pointInfo($point)
    {
        return $point[0] . '_' . $point[1];
    }
 
    /**
     * F值计算：F = G + H
     */
    private function _getF($parent, $point)
    {
        return $this->_getG($parent) + $this->_getH($point);
    }
 
    /**
     * G值计算
     */
    private function _getG($current)
    {
        return isset($this->_g[$this->_pointInfo($current)]) ? $this->_g[$this->_pointInfo($current)] + 1 : 1;
    }
 
    /**
     * H值计算
     */
    private function _getH($point)
    {
        return abs($point[0] - $this->_end[0]) + abs($point[1] - $this->_end[1]);
    }
 
    /**
     * 随机生成路障点数组
     */
    private function _makeDisable()
    {
        if ($this->_num > $this->_x * $this->_y)
            exit('too many disable point');
 
        for ($i = 0; $i < $this->_num; $i++) {
            $tmp = array(
                rand(1, $this->_x),
                rand(1, $this->_y),
            );
            if ($tmp == $this->_start || $tmp == $this->_end || in_array($tmp, $this->disable)) { // 路障点不能与开始点和结束点相同,且路障不重复
                $i--;
                continue;
            }
            $this->disable[] = $tmp;
        }
    }
 
    /**
     * 显示地图
     */
    public function displayPic()
    {
        header('content-type:text/html;charset=utf-8');
        echo '从A到B，绿色背景表示最短路径，黑色背景表示障碍，刷新会重新生成路障。<br /><br />';
        $step = count($this->route);
        echo ($step > 0) ? '<font color="green">共 ' . $step . ' 步</font>' : '<font color="red">无法抵达！</font>';
        echo '<table border="1">';
        for ($y = 1; $y <= $this->_y; $y++) {
            echo '<tr>';
            for ($x = 1; $x <= $this->_x; $x++) {
                $current = array($x, $y);
 
                if (in_array($current, $this->disable)) // 黑色表示路障
                    $bg = 'bgcolor="#000"';
                elseif (in_array($current, $this->route)) // 最短路径
                    $bg = 'bgcolor="#5cb85c"';
                else
                    $bg = '';
 
                if ($current == $this->_start)
                    $content = 'A';
                elseif ($current == $this->_end)
                    $content = 'B';
                else
                    $content = '&nbsp;';
 
                echo '<td style="width:22px; height: 22px;" ' . $bg . '>' . $content . '</td>';
            }
            echo '</tr>';
        }
        echo '</table>';
    }
 
}