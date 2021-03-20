        #region[起跑线]
        //> 1、设置一个栈，用来存储黑色元素。
        //> 、从此行的第0行开始往左扫
        //> 2、如果遇到黑色元素，入栈，
        //> 3、如果遇到白色元素，出栈，出栈的同时统计栈中元素个数，如果栈中元素为4—8个（也就是黑胶的宽度），black_blocks++，否则不加。然后将栈中元素清掉
        //> 4、如果此行的black_blocks在8个左右，times++
        //> 5、遍历18——23行，如果，times在4个左右，则确定为起跑线。
        byte flag_starting_line = 0;
        void check_starting_line()
        {
            //int[] black_nums_stack = new int[20];
            byte times = 0;
            for (byte y = 18; y <= 23; y++)
            {
                byte black_blocks = 0;
                byte cursor = 0;    //指向栈顶的游标
                for (byte x = 0; x <= 185; x++)
                {
                    if (Pixels[y,x] == 0)
                    {
                        if (cursor >= 20)
                        {
                            //当黑色元素超过栈长度的操作   break;    
                        }
                        else 
                        {
                            cursor++;
                        }
                    }
                    else 
                    {
                        if (cursor >= 4 && cursor <= 8)
                        {
                            black_blocks++;
                            cursor = 0;
                        }
                        else 
                        {
                            cursor = 0;
                        }
                    }
                }
                if (black_blocks >= 6 && black_blocks <= 9) times++;
            }
            if (times >= 3 && times <= 5)
            {
                flag_starting_line = 1;
                SetText("进入起跑线状态");
            }
            else 
            {
                flag_starting_line = 0;
            }
        }
        #endregion
