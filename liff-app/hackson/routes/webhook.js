var express = require('express');
var router = express.Router();

// 追加　ミドルウェアの準備すい
var line = require('@line/bot-sdk');
// var middleware = line.middleware; 

router.post('/', function(req, res, next) {
  // 必要な情報追加、アクセストークン
  var client = new line.Client({
    // hackson
    // channelAccessToken: 'X4poG0/kU0K2p3Cp/zcH+y7YIWm7na940B7L0WARviVcb1arERD62iK/SCojWOiGAzS4nc8yX8OhQymQotqC8yF42aPrzZC1hiNazgBA4ox8dygo8gYlsOTfB07la+Xx62r+1hLtdIqw/lyVzdtwc1GUYhWQfeY8sLGRXgo3xvw=',
    // channelSecret : 'b00f5fee0588bfdf8005dcbc48997a1d'
    // things
    channelAccessToken: 'YrPo4XB62RWLElSN13Apn27r56xzN9RDZ5jeMi19keLzPfdWgAAlUIJngDVAD7uMZpIS59BkzwZvBb7ewaOotJYLKk/VQkV6vbbv3Ca82RMnX1a2Zui1GFhXJTUIiIuXjB4pfZ8RujahXW6qjDQzr1GUYhWQfeY8sLGRXgo3xvw=',
    channelSecret : '75cfe39e6eca59eba80d26bba659a26a'
  });
  // 200番を返す
  res.status(200).end();
  // middle使うと「The "data" argument must be one of type string」エラー出るので、公式サイトの部分を参考したらううまでうまく行けた。
  // リクエストの例:https://developers.line.biz/ja/reference/messaging-api/ その後は引き続き、https://qiita.com/TakuTaku04/items/cb71f10669a9e9cbf71bを参考
  var obj = req.body.events[0];
  // eventsの中にuserId、送ってきたメッセージなどすべての情報が入ってる、それを利用して、いろいろ出来そう！～
  console.log(obj);
  // サンプルではpromiseを使ってるけど、ひとまず固定のメッセージを返すのを作る(promiseはhttp通信内包してるから、自分でhttpを送らなくて済むってこと？)
  console.log(obj.message.text);
  console.log(obj.source.userId);
  console.log(obj.type);
  // メッセージが送られてきた場合
  if (obj.type == 'message') {
    // テキストの場合
    if (obj.message.type == 'text') {
      // 名前を、返信メッセージに表示させるのはpromisesが必要そうだが、ひとまず両方ゲットできて、名前をデータベースのIDに使えば、問題ない
      var name = (client.getProfile(obj.source.userId));
      console.log(name);
      client.replyMessage(obj.replyToken, {
        type: "text",
        text: `Chaosさん、${obj.message.text}`
      });
    }
  }

});
  
module.exports = router;